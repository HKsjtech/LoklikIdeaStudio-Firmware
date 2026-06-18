// Copyright (c) 2022 -	Neil
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.
#include "../../Grbl.h" 
#include "../SJCutter.h"
#include "../SJUtils.hpp"
#include "../SJMotorCtrl.h"
#include "../COpticalAlign.h"
#include <OneButton.h>
#include "FS.h"
#include "SPIFFS.h"
#include "../../Uart.h"
#include "../../SettingsDefinitions.h"
#include "../OTAUpdater.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"

SJCutter SJCutterIns;
OneButton btn_power = OneButton(PIN_ONE_KEY, false, false);
OneButton btn_paper;
OneButton btn_start;
OneButton btn_pause;
u_int last_btn_val =0;
// 进纸键模式
u_int paper_btn_mode =PAPER_BTN_MODE::PAPER_BTN_MODE_DEF;
// 记录启动时间
unsigned long start_up_time =millis();

u_int startup_status_tag=0;
u_int startup_status_tags[6]={1000,3000,5000,8000,9000,11000};  

double range[5][2] = {
0,0,
0,0,
0,0,
0,0,
0,0
};	//区间数组
 
float k[4] = {0};	
double ZCommand = 0.0;
float prevPointX = 0.0;	
float prevPointY = 0.0;	


#define BASE26_LEN 26
#define BASE26_STR_LEN 4



// 转换SN为蓝牙ID
void get_formatstr_slice(const char* input,char* delimiter,int index, char* output) {
	char input_str[40];
	memset(input_str,0,40);
	strcat(input_str, input);
	char* token; 
	int i = 0; 
	token = strtok(input_str, delimiter); 
	while (token != NULL) {
		if (i == index) { 
			strcat(output, token);
			break;
		}
		i++;
		token = strtok(NULL, delimiter);
	}
}

// 转换SN为蓝牙ID
void convert_sn_tobtid(char* input, char* output) {
	char* token;
	char* delimiter = "-";
	int i = 0;

	// Use strtok to split the input string into tokens
	token = strtok(input, delimiter);

	// Iterate over the tokens and construct the output string
	while (token != NULL) {
		if (i == 0) {
			// First token should be "G001"
			strcat(output, "RT_");
			strcat(output, token);
		} else if (i >= 2) {
			strcat(output, "_");
			strcat(output, token);
		}
		i++;
		token = strtok(NULL, delimiter);
	}
}

// 转换蓝牙地址为蓝牙ID
void set_btname(char* address) { 
	char sncode[30];
	memset(sncode,0,30);
	sprintf(sncode,"%s",sj_serial_number->get()); 
	{
		char sn_slice1[10];
		char sn_slice2[10];
		char add_slice1[4];
		char add_slice2[4];
		memset(sn_slice1,0,10);
		memset(sn_slice2,0,10);
		memset(add_slice1,0,4);
		memset(add_slice2,0,4);
		get_formatstr_slice(sncode,"-",0,sn_slice1);
		get_formatstr_slice(sncode,"-",3,sn_slice2);
		get_formatstr_slice(address,":",4,add_slice1);
		get_formatstr_slice(address,":",5,add_slice2); 
		memset(sncode,0,30);
		sprintf(sncode,"AT+BTNAME=RT_%s_%s%s_%s\0", sn_slice1,add_slice1,add_slice2,sn_slice2); 
	}
	grbl_sendf(CLIENT_BTSSP,"%s\0",sncode); 
	delay(500);
}


// 将整型编码为字母,最大为ZZZZ
void base26_encode_int(u_int num, char* numstr) {
	int i = BASE26_STR_LEN - 1;
	while (i >= 0) {
		numstr[i] = (char) ('A' + (num % BASE26_LEN));
		num /= BASE26_LEN;
		i--;
	}
	numstr[BASE26_STR_LEN] = '\0';
}

// 将字母解码为整形,最大为456975
u_int base26_decode_int(const char* numstr) {
	u_int num = 0;
	int i;
	for (i = 0; i < BASE26_STR_LEN; i++) {
		num = num * BASE26_LEN + (numstr[i] - 'A');
	}
	return num;
}

void execute_lines(const char* lines_string) { 
	// 拆分文件串为行数组
	char* line = strtok((char*)lines_string, "\n");
	int i = 0;
	while (line != NULL) {
		// 执行命令行
		gc_execute_line(line, CLIENT_SERIAL); 
		// 立即执行缓存
		protocol_buffer_synchronize();
		// 找到下一行
		line = strtok(NULL, "\n");
		i++;
	}
}

void ctrlPanelInit(){
	pinMode(PIN_POWER, OUTPUT);
	pinMode(PIN_MENU_KEY, INPUT);
	pinMode(PIN_LED_POWER, OUTPUT);
	pinMode(PIN_LED_PAUSE, OUTPUT);
	pinMode(PIN_LED_PAPER, OUTPUT);
	pinMode(PIN_LED_START, OUTPUT);
	pinMode(PIN_LED_BLE, OUTPUT);
	pinMode(PIN_LED_USB, OUTPUT);

	pinMode(PEN1_END_STOP, INPUT);
	pinMode(PEN2_END_STOP, INPUT);    
	pinMode(PIN_PAPER, INPUT_PULLDOWN);
	pinMode(X_LIMIT_PIN, INPUT);    
	pinMode(Z_LIMIT_PIN, INPUT_PULLDOWN); 
	pinMode(SEEK_BOX_DISABLE_PIN, OUTPUT);

	digitalWrite(PIN_POWER, HIGH);
	digitalWrite(PIN_LED_POWER, HIGH);
	digitalWrite(PIN_LED_PAUSE, HIGH);
	digitalWrite(PIN_LED_PAPER, HIGH);
	digitalWrite(PIN_LED_START, HIGH);
	digitalWrite(PIN_LED_BLE, HIGH);
	digitalWrite(PIN_LED_USB, HIGH);
	digitalWrite(SEEK_BOX_DISABLE_PIN, LOW);

	SJCutterIns.setCtrlLedAction(BUTTON_POWER,LEDActions::LED_ON);
	SJCutterIns.setCtrlLedAction(BUTTON_PAUSE,LEDActions::LED_ON);
	SJCutterIns.setCtrlLedAction(BUTTON_PAPER,LEDActions::LED_ON);
	SJCutterIns.setCtrlLedAction(BUTTON_START,LEDActions::LED_ON);  

	btn_power.setClickTicks(400);
	btn_power.setPressTicks(800);
	btn_power.attachLongPressStart([](){
		if((millis()-SJCutterIns.powerOnTime)>5000){
			SJCutterIns.powerStatus=0;
			digitalWrite(PIN_POWER, LOW);
			SJCutterIns.setCtrlLedAction(BUTTON_POWER,LEDActions::LED_OFF);
		}
	});

	// btn_pause.setDebounceTicks(80);
	btn_pause.attachClick([](){
		if(SJCutterIns.taskRunStatus)
		{
			SJCutterIns.exchangeTaskPausedStatus(CLIENT_ALL);
		} 
	});

	btn_pause.attachLongPressStart([](){
		if(SJCutterIns.taskRunStatus)
		{
			client_reset_read_buffer(CLIENT_ALL);	//响应立即终止任务操作，需将该通道内所有数据清除
			empty_line(CLIENT_SERIAL);
			empty_line(CLIENT_BTGATT);

			SJMotorCtrl::executeCommand(CLIENT_ALL,"M2");
			protocol_buffer_synchronize();

			// 立即报告状态
			report_realtime_status(CLIENT_ALL); 
			SJMotorCtrl::executeCommand(CLIENT_ALL,"G0Z0F5000");
			char cmdLine[30];
			memset(cmdLine,0,30);
			// X轴机械结构拉出距离
			float x_pulloff_dist = sj_cfg_x_pulloff_dist->get();
			sprintf(cmdLine, "G0 X%0.2fY0",x_pulloff_dist);
			SJMotorCtrl::executeCommand(CLIENT_ALL,cmdLine);
			protocol_buffer_synchronize();

			memset(cmdLine,0,30);
			sprintf(cmdLine, "G0 Y%.2f",DEFAULT_Y_OFFSET);
			SJMotorCtrl::executeCommand(CLIENT_ALL,cmdLine);
			SJMotorCtrl::executeCommand(CLIENT_ALL,"M2");
			protocol_buffer_synchronize(); 
		}
		// 任务清除
		SJCutterIns.taskPausedStatus=false;
		SJCutterIns.taskRunStatus=false;
		SJCutterIns.taskSecurityCode=0;
		SJCutterIns.taskRecvLine=0;
		SJCutterIns.mateLoadStatus=true;
		SJCutterIns.resetButtonST();
		
		grbl_msg_sendf(CLIENT_ALL,MsgLevel::Info,"Program Clear");
	});

	// btn_paper.setDebounceTicks(80);
	btn_paper.setClickMs(1000);
	btn_paper.setPressTicks(1000);
	btn_paper.attachClick([](){
		if(SJCutterIns.taskRunStatus)return;
		SJCutterIns.exchangeMateLoadStatus(CLIENT_ALL);  
	});

	// btn_start.setDebounceTicks(80);
	btn_start.attachClick([](){
		if(paper_btn_mode==PAPER_BTN_MODE::PAPER_BTN_MODE_ADJ){
			SJMotorCtrl::moveAndResetYAxis(CLIENT_ALL,-10);
		}else {
			if(SJCutterIns.testMode){
				// 结束测试模式
				SJCutterIns.setCtrlLedAction(BUTTON_START, LEDActions::LED_OFF);
				SJCutterIns.testMode=false;
			}else{
				if(!SJCutterIns.taskRunStatus&&!SJCutterIns.taskPausedStatus){
					SJCutterIns.setButtonST(BUTTON_START,false);
					SJCutterIns.setCtrlLedAction(BUTTON_START, LEDActions::LED_OFF);
					// 立即报告状态
					report_realtime_status(CLIENT_ALL); 
				}  
				// COpticalAlign optalign;
				// SJMotorCtrl::xyAxisHoming(CLIENT_ALL,false);
				// optalign.reviseOffset(1, 0);
				// SJMotorCtrl::executeCommand(CLIENT_ALL,"$J=G90G21X150Y0F5000");
			} 
		} 
	});

	btn_start.attachLongPressStart([](){
		if(SJCutterIns.taskRunStatus || SJCutterIns.taskPausedStatus || SJCutterIns.testMode)return;
		EEPROM.writeByte(EEPROM_ADDRESS,0xAA);
		delay(10);
		EEPROM.commit();
		ESP.restart();
	});

	// btn_start.attachMultiClick([](){
	// 	if(SJCutterIns.taskRunStatus)return;
	// 		paper_btn_mode=PAPER_BTN_MODE::PAPER_BTN_MODE_DEF;
	// 	if(!SJCutterIns.testMode) {
	// 		SJCutterIns.setCtrlLedAction(BUTTON_START, LEDActions::LED_FAST);
	// 		SJMotorCtrl::runTest(CLIENT_ALL);
	// 		// SJMotorCtrl::runTest_Rectangle(CLIENT_ALL);
	// 	}        
	// 		SJCutterIns.setCtrlLedAction(BUTTON_START, LEDActions::LED_OFF); 
	// 	});
}

portMUX_TYPE transMutex = portMUX_INITIALIZER_UNLOCKED;
unsigned long lastUsbEnterTime = 0;
unsigned long lastBlueEnterTime = 0;
unsigned long usbEnterTime = 0;
unsigned long middleUsbEnterTime = 0;
unsigned long blueEnterTime = 0;
unsigned long middleBlueEnterTime = 0;
void SJCutter::cheakLedConnectStatus(uint8_t client)
{
	if(client == CLIENT_SERIAL)
	{
		vTaskEnterCritical(&transMutex);
		lastUsbEnterTime = millis();
		vTaskExitCritical(&transMutex);
		if(this->ledSet[BUTTON_USB][1] == LEDActions::LED_OFF)
		{
			SJCutterIns.setCtrlLedAction(BUTTON_USB,LEDActions::LED_ON);
			BLEGATTServer::getInstance().setBleFindStatus(false);
		}
	}
	if(client == CLIENT_BTGATT && !this->taskRunStatus)
	{
		vTaskEnterCritical(&transMutex);
		lastBlueEnterTime = millis();
		vTaskExitCritical(&transMutex);
		if(this->ledSet[BUTTON_BLUE][1] == LEDActions::LED_OFF)
		{
			digitalWrite(PIN_LED_BLE, HIGH);
			SJCutterIns.setCtrlLedAction(BUTTON_BLUE,LEDActions::LED_ON);
		}
	}
	if(sys.state == State::Alarm && !SJCutterIns.xHomeStatus)
		grbl_sendf(client, "error:%d\n", static_cast<int>(Error::XHomeFile));
	if(sys.state == State::Alarm && !SJCutterIns.zHomeStatus)
		grbl_sendf(client, "error:%d\n", static_cast<int>(Error::ZHomeFile));
}

void SJCutter::cheakLedDisConnectStatus()
{
	if(this->ledSet[BUTTON_USB][1] == LEDActions::LED_ON)
	{
		middleUsbEnterTime = lastUsbEnterTime;
		usbEnterTime = millis();
		if((usbEnterTime - middleUsbEnterTime) > HEARTBEAT_TIMEOUT_MS)
		{
			SJCutterIns.setCtrlLedAction(BUTTON_USB,LEDActions::LED_OFF);
			SJCutterIns.TaskClear(CLIENT_SERIAL);
			BLEGATTServer::getInstance().setBleFindStatus(true);
		}
	}
	if(this->ledSet[BUTTON_BLUE][1] == LEDActions::LED_ON)
	{
		middleBlueEnterTime = lastBlueEnterTime;
		blueEnterTime = millis();
		if(((blueEnterTime - middleBlueEnterTime) > HEARTBEAT_TIMEOUT_MS && !this->taskRunStatus) || !BLEGATTServer::getInstance().hasClient()) // 有任务的时候不能切换蓝牙灯状态，会产生一个奇怪的bug，服了  
		{
			SJCutterIns.setCtrlLedAction(BUTTON_BLUE,LEDActions::LED_OFF);
			SJCutterIns.TaskClear(CLIENT_BTGATT);
		}
	}
}

void ctrlPanelLoop(){
	unsigned long now = millis();
	unsigned long now_mic = micros();
	////////////////////////////////////////////////////////////////////////////
	// 按钮状态检测 
	btn_power.tick();
	// 当设备处于就绪状态时才能接收外部事件
	if(SJCutterIns.readyStatus){
		// 功能按钮
		int tValue = analogRead(PIN_MENU_KEY);
		if(tValue>=0&&tValue<2200){
			last_btn_val=tValue;      
		}
		// btn_pause.tick((tValue > 2000 && tValue < 3000));
		// btn_paper.tick((tValue > 1000 && tValue < 2000));
		// btn_start.tick((tValue > 300 && tValue < 1000));
		btn_paper.tick((tValue > 1500 && tValue < 2200));
		btn_start.tick((tValue > 500 && tValue < 1200));
		btn_pause.tick((tValue >= 0 && tValue <20));         
	}
	//////////////////////////////////////////////////////////////////////////////
	// 启动初化动作
	// startup_status_tag=0  start_up_time= millis()
	if(startup_status_tag<6){	
		// 判断系统状态是否允许执行
		bool able_run_c1=!(sys.state == State::Alarm || sys.state == State::Jog ||  sys.state == State::Homing);
		// 判断在时间范围内
		// bool able_run_c2=((now-start_up_time)>startup_status_tags[startup_status_tag]);

		if(startup_status_tag==0&&able_run_c1){	//&&able_run_c2
			SJMotorCtrl::zAxisHoming1(CLIENT_ALL);
			// SJMotorCtrl::zAxisHoming(CLIENT_ALL);
			startup_status_tag++;
		}else if(startup_status_tag==1&&able_run_c1){
			SJMotorCtrl::moveYAxis(CLIENT_ALL,-320);
			startup_status_tag++;      
		}else if(startup_status_tag==2&&able_run_c1){
			SJMotorCtrl::resetAxis(CLIENT_ALL,true,true,true);
			startup_status_tag++;
		}else if(startup_status_tag==3&&able_run_c1){
			SJCutterIns.setCtrlLedAction(BUTTON_PAUSE,LEDActions::LED_OFF);
			SJCutterIns.setCtrlLedAction(BUTTON_PAPER,SJCutterIns.mateLoadStatus?LEDActions::LED_ON:LEDActions::LED_OFF);				
			SJCutterIns.setCtrlLedAction(BUTTON_START,LEDActions::LED_OFF);  
			SJCutterIns.setButtonST(BUTTON_POWER,true);
			SJCutterIns.setButtonST(BUTTON_PAUSE,false);
			SJCutterIns.setButtonST(BUTTON_PAPER,SJCutterIns.mateLoadStatus);			
			SJCutterIns.setButtonST(BUTTON_START,false);
			digitalWrite(PIN_LED_BLE, LOW);
			digitalWrite(PIN_LED_USB, LOW);
			SJCutterIns.taskRunStatus=false;
			SJCutterIns.taskPausedStatus=false;
		//   SJCutterIns.mateLoadStatus=true;    
		//   SJCutterIns.exchangeMateLoadStatus(CLIENT_ALL);
			startup_status_tag++;
		}else if(startup_status_tag==4&&able_run_c1){
			// grbl_sendf(CLIENT_ALL,"[MSG]\r\n");
			grbl_msg_sendf(CLIENT_ALL,MsgLevel::Info,"Ready");
			SJCutterIns.readyStatus=true;
			startup_status_tag++;
		}else if(startup_status_tag==5){
			// 	  Debug
			//    SJCutterIns.readyStatus=true;
			//    SJCutterIns.taskRunStatus=false;
			//    SJCutterIns.taskPausedStatus=false;
			//    SJCutterIns.mateLoadStatus=true;
			//    startup_status_tag++;
			//    grbl_msg_sendf(CLIENT_ALL,MsgLevel::Info,"Ready");
		}
	}
	// 灯光控制
	for (int i = 1; i < 6; i++){
		int key = SJCutterIns.ledSet[i][0];
		int action = SJCutterIns.ledSet[i][1];
		if (action == LEDActions::LED_OFF){
			digitalWrite(key, LOW);
		}else if (action == LEDActions::LED_ON){
			digitalWrite(key, HIGH);
		}else if (action == LEDActions::LED_SLOW){
			if (now % 1000 < 500){
				digitalWrite(key, LOW);
			}else{
				digitalWrite(key, HIGH);
			}
		}else if (action == LEDActions::LED_FAST){
			if (now % 200 < 100){
				digitalWrite(key, LOW);
			}else{
				digitalWrite(key, HIGH);
			}
		}
	}
}

void SJCutter::reboot(){
	startup_status_tag=0;
	this->readyStatus=false;
}

// 指令处理器
CMDResult SJCutter::handleMessage(uint8_t client,char* line) {
	if(!SJCutterIns.readyStatus){
		return CMDResult::RESULT_ABANDON; 
	}

	// 等待可用状态
	//   if(!SJMotorCtrl::waitRunStatus()){
	//   return CMDResult::RESULT_ABANDON; 
	// }

	if(client == CLIENT_SERIAL)
	{
		vTaskEnterCritical(&transMutex);
		lastUsbEnterTime = millis();
		vTaskExitCritical(&transMutex);
	}
	if(client == CLIENT_BTGATT)
	{
		vTaskEnterCritical(&transMutex);
		lastBlueEnterTime = millis();
		vTaskExitCritical(&transMutex);		//刷新连接状态
	}
		
	// 处理编码数据
	CMDResult proc_status=this->securityProcess(client,line);
	if(proc_status==CMDResult::RESULT_ABANDON){
		return CMDResult::RESULT_ABANDON;
	}
	// 任务执行状态
	bool task_status=this->taskRunStatus;
	// 指令执行消息响应
	char response_msg[64];
	memset(response_msg,0, 64); 

	// 替换g指令速度参数
	char* f_pos = strchr(line, 'F');
	if (f_pos == NULL){
		if (!strncmp(line, "G0",2)) {  
			float x = 0.0;
			float y = 0.0;
			sscanf(line,"G0X150Y%f",&y);
			if(y > 39)
			{
				this->smartMaterial = true;
			}
			// 调整Z轴，增加补偿值
			this->adjustZValue(true, line);
			// 替换快速移动速度
			int data_len=strlen(line);
			char speedconfig[10];
			memset(speedconfig,0, 10);   
			sprintf(speedconfig, "F%d", this->moveSpeed);
			memcpy(line+data_len,speedconfig,10); 
			// grbl_sendf(CLIENT_SERIAL, "GCODE:%s\r\n",line);
		}else if (!strncmp(line, "G1",2)) {
			float x = 0.0;
			float zCompen = 0.0;
			sscanf(line,"G1X%f",&x);
			zCompen = this->returnCompen(x);	//先进行区域补偿
			char zConfig[10];
			memset(zConfig,0, 10);
			sprintf(zConfig,"Z%.2f",ZCommand - zCompen);
			int data_len=strlen(line);
			strncpy(line+data_len,zConfig,10); 	//将补偿结果添加至字符串末尾
			// grbl_sendf(CLIENT_SERIAL, "GCODE:%s\r\n",line);

			// 调整Z轴，增加补偿值
			this->adjustZValue(false, line);
			// 替换工作移动速度
			data_len=strlen(line);
			char speedconfig[10];
			memset(speedconfig,0, 10);   
			sprintf(speedconfig, "F%d", (this->taskSpeed / 1000) * ((u_int)DEFAULT_X_MAX_RATE / 10));		
			memcpy(line+data_len,speedconfig,10); 
			// grbl_sendf(CLIENT_SERIAL, "GCODE:%s\r\n",line);
		}
	}

	// if (!strncmp(line, "G92",3)) {
	//      // 定义工作坐标，G92X{0}Y{1}Z{2}
	//   float work_coord_x_offset=homing_pulloff->get()+sj_cfg_x_offset->get();
	//   // float work_coord_x_offset=homing_pulloff->get()+DEFAULT_HOMING_OFFSET+sj_cfg_x_offset->get();
	//   float work_coord_y_offset=sj_cfg_y_offset->get();

	//   char cmdLine[30];
	//   memset(cmdLine,0,30);
	//   sprintf(cmdLine, "G92X%0.2fY%0.2fZ0",work_coord_x_offset,work_coord_y_offset);
	//   memcpy(line,cmdLine,30); 
	//   // SJMotorCtrl::executeCommand(client,cmdLine);
	//   // proc_status=CMDResult::RESULT_IGNORE; 
	// }

	//  grbl_sendf(CLIENT_SERIAL, "GCODE:%d|%s\r\n",this->waitForStatus(),line);

	// 受状态控制的  ||strncmp(line, "BR:",3)
	if ((!strncmp(line, "G",1)&&!this->waitForStatus())) { 
		// 当任务未启动时拒绝G指令的执行
		proc_status=CMDResult::RESULT_ABANDON; 
	}else if (!strcmp(line, "HP")) { 
		// Z轴归位
		bool result= SJMotorCtrl::zAxisHoming1(client); 
		// this->resetButtonST();
		proc_status=result?CMDResult::RESULT_RESPONSE_OK:CMDResult::RESULT_RESPONSE_FAIL; 
	}else if (!strcmp(line, "HA")) { 
		// 归位
		bool result=SJMotorCtrl::resetAxis(CLIENT_ALL,true,true,true);
		// this->resetButtonST();
		proc_status=result?CMDResult::RESULT_RESPONSE_OK:CMDResult::RESULT_RESPONSE_FAIL;  
	}else if (!strcmp(line, "HPA")) { 
	// 归位，对刀,X回中间
		bool result=SJMotorCtrl::zAxisHoming1(client);
		// bool result=SJMotorCtrl::zAxisHoming(client);
		result=result&&SJMotorCtrl::resetAxis(CLIENT_ALL,true,true,true);
		proc_status=result?CMDResult::RESULT_RESPONSE_OK:CMDResult::RESULT_RESPONSE_FAIL;      
	}else if (!strcmp(line, "HPX")) { 
	// 归位，对刀
		bool result=SJMotorCtrl::zAxisHoming1(client);
		// bool result=SJMotorCtrl::zAxisHoming(client);
		result=result&&SJMotorCtrl::resetAxis(CLIENT_ALL,true,true,false);
		proc_status=result?CMDResult::RESULT_RESPONSE_OK:CMDResult::RESULT_RESPONSE_FAIL;        
	}else if (!strcmp(line, "HAX")) { 
	// 对刀，X回零位
		bool result=SJMotorCtrl::resetAxis(CLIENT_ALL,false,false,false); 
		// this->resetButtonST();
		proc_status=result?CMDResult::RESULT_RESPONSE_OK:CMDResult::RESULT_RESPONSE_FAIL; 
	}else if (!strncmp(line, "MA Z",4)) { 
		int movedist=0;
		sscanf(line + 4, "%d", &movedist);
		SJMotorCtrl::absMoveAxis(client,Z_AXIS,movedist);
		proc_status=CMDResult::RESULT_OK; 
	}else if (!strncmp(line, "MA X",4)) { 
		int movedist=0;
		sscanf(line + 4, "%d", &movedist);
		SJMotorCtrl::absMoveAxis(client,X_AXIS,movedist);
		proc_status=CMDResult::RESULT_OK; 
	}else if (!strncmp(line, "MA Y",4)) { 
		int movedist=0;
		sscanf(line + 4, "%d", &movedist);
		SJMotorCtrl::absMoveAxis(client,Y_AXIS,movedist);
		proc_status=CMDResult::RESULT_OK; 
	}else if (!strncmp(line, "MR Y",4)) { 
		int movedist=0;
		sscanf(line + 4, "%d", &movedist);
		SJMotorCtrl::relMoveAxis(client,Y_AXIS,movedist);
		proc_status=CMDResult::RESULT_OK; 
	}else if (!strcmp(line, "SV")) {
		char status_info_pkg[1024];
		memset(status_info_pkg,0,1024);
		// 获取传感器     
		this->readStatus((char*)status_info_pkg,1);
		grbl_sendf(client, "STATUS:%s\r\n",status_info_pkg);
		proc_status=CMDResult::RESULT_ABANDON; 
	}else if (!strncmp(line, "TS",2)) {     
		// 任务开始前强制等待状态
		this->waitForStatus();
		// 任务开始前强制改变按钮状态
		this->resetButtonST();
		// 任务开始    
		this->taskRunStatus=true;  
		this->taskPausedStatus=false; 

		// 定义工作坐标，G92X{0}Y{1}Z{2}
		float work_coord_x_offset=homing_pulloff->get()+ DEFAULT_X_OFFSET;
		// float work_coord_x_offset=homing_pulloff->get()+DEFAULT_HOMING_OFFSET+sj_cfg_x_offset->get();
		float work_coord_y_offset= DEFAULT_Y_OFFSET;

		char cmdLine[30];
		memset(cmdLine,0,30);
		sprintf(cmdLine, "G92X%0.2fY%0.2fZ0",work_coord_x_offset,work_coord_y_offset);
		SJMotorCtrl::executeCommand(client,cmdLine);

		this->calcuRangeCompen();	//计算区域压力补偿
		paper_btn_mode=PAPER_BTN_MODE::PAPER_BTN_MODE_DEF;
		grbl_msg_sendf(client,MsgLevel::Info,"Program Run");
		proc_status=CMDResult::RESULT_OK; 
	}else if (!strcmp(line, "TE")) {
		// 任务终止  

		if(this->smartMaterial == false)
		{
			char cmdLine[30];
			memset(cmdLine,0,30);
			// float y_offset = sj_cfg_y_offset->get();
			sprintf(cmdLine, "G0 Y%.2f",DEFAULT_Y_OFFSET);
			SJMotorCtrl::executeCommand(client,cmdLine);
			protocol_buffer_synchronize();
		}

		this->taskRunStatus=false;  
		this->taskSecurityCode=0;
		this->taskRecvLine=0;  
		// grbl_msg_sendf(client,MsgLevel::Info,"TaskEnd");
		proc_status=CMDResult::RESULT_OK; 
	}else if (!strncmp(line, "TF ",3)) { 
		u_int wspeed=5000;
		u_int mspeed=7000;
		sscanf(line + 3, "%d,%d", &wspeed,&mspeed); 
		SJCutterIns.taskSpeed=wspeed;
		SJCutterIns.moveSpeed=mspeed;
		proc_status=CMDResult::RESULT_OK; 
	}else if (!strcmp(line, "TP")) {
		// 任务暂停  
		this->taskPausedStatus=true;  
		SJCutterIns.setCtrlLedAction(BUTTON_PAUSE,LEDActions::LED_ON);  
		SJCutterIns.setButtonST(BUTTON_PAUSE,true);
		SJCutterIns.setCtrlLedAction(BUTTON_START,LEDActions::LED_OFF);  
		SJCutterIns.setButtonST(BUTTON_START,false);
		// grbl_msg_sendf(client,MsgLevel::Info,"Program Pause");
		// 立即报告状态
		report_realtime_status(CLIENT_ALL);
		proc_status=CMDResult::RESULT_RESPONSE_OK; 
	}else if (!strcmp(line, "TU")) {
		// 任务恢复
		this->taskPausedStatus=false;  
		SJCutterIns.setCtrlLedAction(BUTTON_PAUSE,LEDActions::LED_OFF);  
		SJCutterIns.setButtonST(BUTTON_PAUSE,false);
		// grbl_msg_sendf(client,MsgLevel::Info,"Program UnPause");
		// 立即报告状态
		report_realtime_status(CLIENT_ALL);
		proc_status=CMDResult::RESULT_RESPONSE_OK;      
	}else if (!strcmp(line, "TC")) {
		//  protocol_execute_realtime();
		if(this->taskRunStatus){
			
			SJMotorCtrl::executeCommand(client,"G0Z0F5000");
			char cmdLine[30];
			memset(cmdLine,0,30);
			// X轴机械结构拉出距离
			float x_pulloff_dist = sj_cfg_x_pulloff_dist->get();
			sprintf(cmdLine, "G0 X%0.2fY0",x_pulloff_dist);
			SJMotorCtrl::executeCommand(client,cmdLine);
			protocol_buffer_synchronize();
			memset(cmdLine,0,30);
			// float y_offset = sj_cfg_y_offset->get();
			sprintf(cmdLine, "G0 Y%.2f",DEFAULT_Y_OFFSET);
			SJMotorCtrl::executeCommand(client,cmdLine);
			// SJMotorCtrl::executeCommand(client,"G92.1");
			// protocol_buffer_synchronize();
			SJMotorCtrl::executeCommand(client,"M2");
			protocol_buffer_synchronize();

			
			// SJMotorCtrl::moveYAxis(CLIENT_ALL,-320);
			// SJMotorCtrl::resetAxis(CLIENT_ALL,true,true,true);
			
		}
			// 任务清除
		this->taskPausedStatus=false;  
		this->taskRunStatus=false;  
		this->taskSecurityCode=0;
		this->taskRecvLine=0;
		// this->mateLoadStatus=true;  
		this->resetButtonST();
		// if(!this->taskRunStatus){
		//   this->exchangeMateLoadStatus(client);
		// }

		grbl_msg_sendf(client,MsgLevel::Info,"Program Clear");
		proc_status=CMDResult::RESULT_RESPONSE_OK; 
	}else if (!strcmp(line, "LM")) {
		// 加载物料
		this->mateLoadStatus=false;  
		this->exchangeMateLoadStatus(client); 
		proc_status=CMDResult::RESULT_RESPONSE_OK; 
	}else if (!strcmp(line, "UM")) {
		// 退出物料
		this->mateLoadStatus=true;  
		this->exchangeMateLoadStatus(client); 
		proc_status=CMDResult::RESULT_RESPONSE_OK; 
	}else if (!strcmp(line, "TESTA")) {
		char file_string[] ="G0 X1 F5000\nG0 X150 F5000\nG0 X1 F5000\nG0 X100 F5000";
		execute_lines(file_string);
		proc_status=CMDResult::RESULT_RESPONSE_OK; 
	}else if (!strcmp(line, "TESTB")) {
		grbl_sendf(CLIENT_ALL, "LOST_LINE:%d\r\n", digitalRead(Z_LIMIT_PIN));
		proc_status=CMDResult::RESULT_OK;  
	}else if (!strncmp(line, "TESTOA", 6)) {
		// 机器状态控制 
		float max_val=6.0;
		int axio_dir=0;
		int axio_speed=9000;
		sscanf(line + 7, "%f,%d,%d", &max_val,&axio_dir,&axio_speed); 
		grbl_sendf(CLIENT_SERIAL, "PARAMS: %0.2f,%d,%d\r\n", max_val,axio_dir,axio_speed);   
		SJMotorCtrl::TestFindPointByRange(max_val,axio_dir,axio_speed); 
		proc_status=CMDResult::RESULT_OK; 
	}else if (!strcmp(line, "$BC=AT+BRATE")) { 
		char blueconfig[20];
		memset(blueconfig,0, 20);   
		sprintf(blueconfig, "%s", "AT+BRATE=115200");
		Uart2.close();
		Uart2.setPins(GPIO_NUM_16,GPIO_NUM_17);
		Uart2.begin(BLE_BAUD_RATE, Uart::Data::Bits8, Uart::Stop::Bits1, Uart::Parity::None);
		grbl_send(CLIENT_BTSSP,blueconfig);
		proc_status=CMDResult::RESULT_OK; 
	}else if (!strcmp(line, "$BC=AT+BTNAME")) {  
		grbl_sendf(CLIENT_BTSSP,"AT+BTNAME?\0");
		proc_status=CMDResult::RESULT_ABANDON;
	}else if (!strncmp(line, "$BC=", 4)) {
		char blueconfig[20];
		memset(blueconfig,0, 20);    
		// 配置蓝牙 
		sscanf(line + 4, "%s", &blueconfig);
		grbl_sendf(CLIENT_BTSSP,"%s\0",blueconfig);
		proc_status=CMDResult::RESULT_ABANDON; 
	}else if (!strncmp(line, "$BLE_ADDR", 9)) { 
		grbl_sendf(CLIENT_BTSSP,"AT+LADDR?\0");
		proc_status=CMDResult::RESULT_ABANDON;
	}else if (!strncmp(line, "$SN_RESET=", 10)) {
		// 重写入序列号
		sj_serial_number->setStringValue(line + 10);   
		// 标记已经更新
		this->snUpdateTag =true;
		grbl_sendf(CLIENT_BTSSP,"AT+DISCON\0");
		delay(500);
		// 向蓝牙模块发起地址查询
		grbl_sendf(CLIENT_BTSSP,"AT+LADDR?\0");
		proc_status=CMDResult::RESULT_OK; 
	}else if (!strncmp(line, "$SN=", 4)) {
		auto sn=sj_serial_number->get();
		// 仅当初始编码允许修改
		if(sn==DEFAULT_SERIAL_NUMBER){
			// 写入序列号
			sj_serial_number->setStringValue(line + 4);
			// 标记已经更新
			this->snUpdateTag =true;
			grbl_sendf(CLIENT_BTSSP,"AT+DISCON\0");
			delay(500);
			// 向蓝牙模块发起地址查询
			grbl_sendf(CLIENT_BTSSP,"AT+LADDR?\0");
			proc_status=CMDResult::RESULT_OK; 
		}else{
			proc_status=CMDResult::RESULT_FAIL; 
		} 
	}else if (!strcmp(line, "$SN")) {
		// 设置序列号
		this->readSerialNumber(client); 
		proc_status=CMDResult::RESULT_ABANDON;    
	}else if (!strcmp(line, "$FV")) {
		// 读取固件版本
		this->readFirmwareVersion(client);
		proc_status=CMDResult::RESULT_ABANDON; 
	}else if (!strncmp(line, "$CMV", 4)) {// 待优化区分！！！！！
		if(strlen(line) < 7)
		{
			// 查询机器XYZ三轴步长参数
			this->readStepsPerMM(client);
			proc_status=CMDResult::RESULT_ABANDON; 
		}
		else
		{
			// 查询机器XY三轴步长参数和偏移
			this->writeStepsPerMM(client,line);
			proc_status=CMDResult::RESULT_OK;   
		}  
	}
	else if (!strncmp(line, "$CZV",4)) {
		// 查询机器Z三轴步长参数
		this->writeZOffset(client,line);
		proc_status=CMDResult::RESULT_OK;  
	}else if (!strncmp(line, "$MSC",4)) {
		// 机器状态控制
		int key_val=-1;
		int state_val=-1;
		sscanf(line + 5, "%d,%d", &key_val,&state_val);
		grbl_sendf(CLIENT_SERIAL,"%s\n",line);
		if(key_val>=0&&key_val<=3&&(state_val>=0&&state_val<=3)){      
			this->setButtonST(key_val,state_val>0);
			this->setCtrlLedAction(key_val,state_val);
			proc_status=CMDResult::RESULT_OK;       
		}else{
			proc_status=CMDResult::RESULT_FAIL;  
		}
	}else if (!strcmp(line, "XUN1")) {
		int a = digitalRead(SEEK_BOX_DISABLE_PIN);
		grbl_sendf(client, "val:%d\n", a);
		digitalWrite(SEEK_BOX_DISABLE_PIN,HIGH);
		proc_status=CMDResult::RESULT_OK;
	}else if (!strncmp(line, "SEEKMARK", 8)&&!task_status) {
		this->setCtrlLedAction(BUTTON_START,false);
		digitalWrite(PIN_LED_START, LOW);
		digitalWrite(SEEK_BOX_DISABLE_PIN,HIGH);
		//设置序列号
		int   tTaskIndex = 0;
		float tAreaW     = 0;
		float tAreaH     = 0;
		int param_index = 0;// 巡边校准参数索引

		// 从参数中获取参数
		sscanf(line + 9, "%d,%f,%f,%d", &tTaskIndex, &tAreaW, &tAreaH, &param_index);
		COpticalAlign optalign;
		// SJMotorCtrl::xyAxisHoming(client,false);
		SJMotorCtrl::executeCommand(client,"$HX");
		// 等待可用状态
		SJMotorCtrl::waitRunStatus();

		SJCutterIns.readyStatus=false;
		optalign.reviseOffset(tTaskIndex, param_index);
		// SJMotorCtrl::resetAxis(CLIENT_ALL,true,true,true);
		// 定义工作坐标，G92X{0}Y{1}Z{2}
		float work_coord_x_offset=homing_pulloff->get()+DEFAULT_X_OFFSET;
		// float work_coord_x_offset=homing_pulloff->get()+DEFAULT_HOMING_OFFSET+sj_cfg_x_offset->get();
		float work_coord_y_offset=DEFAULT_Y_OFFSET;

		char cmdLine[30];
		memset(cmdLine,0,30);
		sprintf(cmdLine, "G92X%0.2fY%0.2fZ0",work_coord_x_offset,work_coord_y_offset);
		SJMotorCtrl::executeCommand(client,cmdLine);
		protocol_buffer_synchronize();

		SJMotorCtrl::executeCommand(client,"$HX");
		protocol_buffer_synchronize();

		memset(cmdLine,0,30);
		// X轴机械结构拉出距离
		float x_pulloff_dist = sj_cfg_x_pulloff_dist->get();
		sprintf(cmdLine, "G0 X%0.2fY%0.2f",x_pulloff_dist, work_coord_y_offset);
		SJMotorCtrl::executeCommand(client,cmdLine);
		protocol_buffer_synchronize();

		SJCutterIns.readyStatus=true;
		digitalWrite(SEEK_BOX_DISABLE_PIN,LOW);
		proc_status=CMDResult::RESULT_OK;
	}else if (!strncmp(line, "TEST",4)) { 
		// 单轴测试
		char newCommand[20];  
		char *cmdPos;  
		cmdPos = strchr(line, ' ');  
		if (cmdPos != NULL) {  
			// 构造慢跑指令 
			snprintf(newCommand, sizeof(newCommand), "$J=G91G21%s", cmdPos+1);  
			// grbl_send(client,newCommand);
			SJMotorCtrl::executeCommand(client,newCommand);
		}  
		proc_status=CMDResult::RESULT_ABANDON; 
	}else if(!strncmp(line, "$CMMV ",6)) {
		//写入区域压力补偿
		this->writeRangeCompen(line);
		proc_status=CMDResult::RESULT_OK; 
	}else if(!strncmp(line, "TZ ",3))		//TZ G0Z-60
	{
		// sscanf(line+6,"%lf",&ZCommand);
		proc_status=CMDResult::RESULT_OK;
	} else if(!strncmp(line, "TM 1",4)) {	//蓝牙块传输校验模式产生标记;
	// 获取 BLEGATTServer 的唯一实例
  	BLEGATTServer& u1BLE = BLEGATTServer::getInstance();
	u1BLE.otaMode = 2;
	proc_status=CMDResult::RESULT_RESPONSE_OK;
	// grbl_sendf(CLIENT_SERIAL,"otaMode=2.\n");
  } else if(!strncmp(line, "TM 0",4)) {	//蓝牙块传输校验模式产生标记;
	// 获取 BLEGATTServer 的唯一实例
  	BLEGATTServer& u1BLE = BLEGATTServer::getInstance();
	u1BLE.otaMode = 0;
	proc_status=CMDResult::RESULT_RESPONSE_OK;
	// grbl_sendf(CLIENT_SERIAL,"otaMode=0.\n");
  } else if(!strncmp(line, "RPA", 3)) {  //Running partition address 获取当前固件运行的分区信息
    const esp_partition_t *running_partition = esp_ota_get_running_partition();
    if (running_partition != NULL) {
        grbl_sendf(client,"RPA:0x%08x\n", running_partition->address);
		proc_status=CMDResult::RESULT_RESPONSE_OK;
    }
  }

	if(proc_status==CMDResult::RESULT_IGNORE){
		// 判断来源为蓝牙
		if(client==CLIENT_BTSSP){
			//grbl_sendf(CLIENT_SERIAL,"BR: %s\r\n",line);    
			// 判断为配置返回
			if (!strncmp(line, "+",1)) {
				//判断是否为等待更新为状态this->snUpdateTag&&
				if (this->snUpdateTag&&!strncmp(line, "+LADDR=",7)) {
				char btadd[40] = "";
				memset(btadd,0,40);
				sscanf(line + 7, "%s", &btadd); 
				set_btname(btadd); 
				}else{
				// 直接转发至串口
				grbl_sendf(CLIENT_ALL,"BR: %s\r\n",line); 
				}
				proc_status=CMDResult::RESULT_ABANDON;
			}
			
		}
		// 修改输入指令
		this->modifyCommendLine(line);    
	}

	// 劫持响应类返回结果，并重置为抛弃返回
	if(proc_status==CMDResult::RESULT_RESPONSE_OK){
		grbl_sendf(client,"RSP_OK:%s\r\n",line); 
		proc_status=CMDResult::RESULT_ABANDON; 
	}else if(proc_status==CMDResult::RESULT_RESPONSE_FAIL){    
		sprintf(response_msg, "%d\0",1);
		grbl_sendf(client,"RSP_FAIL:%s,%s\r\n",line,response_msg); 
		proc_status=CMDResult::RESULT_ABANDON; 
	}
	return proc_status; 
}

SJCutter::SJCutter() {}
SJCutter::~SJCutter() {}

void SJCutter::init() {    
	if (!SPIFFS.exists("/")){
		SPIFFS.begin(true);
	}  
	SJMotorCtrl::init();    
}

bool test_flag = false;
// 事务循环
void SJCutter::loop() {    

	SJMotorCtrl::loop();

	ctrlPanelLoop(); 
	SJCutterIns.cheakLedDisConnectStatus();
}

// 设置按钮状态
void SJCutter::setButtonST(u_int key,bool val){
	u_int oldval=this->buttonSet[key][1]; 
	u_int newval=val?1:0;
	if(oldval!=newval){
		this->buttonSet[key][1]=newval;
		// this->ledSet[key][1]=newval;
	}
}

	// 等待状态
bool SJCutter::waitForStatus(){
	unsigned long starttime = millis();
	unsigned long waittime = millis();
	bool tag=(sys.state==State::Homing||sys.state==State::Jog);
	int acount=0;
	while (tag)
	{
		tag=(sys.state==State::Homing||sys.state==State::Jog);
		if((!tag)&&((millis()-waittime)>2000)){
			break;
		}else if(tag){
			waittime = millis();
			// if((millis()-waittime)>2000){
			//   // sys_rt_exec_state.bit.motionCancel = true;
			//   SJMotorCtrl::executeRealtimeCommand(CLIENT_ALL,0x85);
			//   break;
			// }
		}
		yield();
		acount++;
	}
	// grbl_sendf(CLIENT_ALL,"waitForStatus COUNT: %d, %d\r\n", acount,sys.state);
	return this->taskRunStatus;  
}

// 重置按钮状态
void SJCutter::resetButtonST(){

	this->setButtonST(BUTTON_PAPER,this->mateLoadStatus);
	this->setCtrlLedAction(BUTTON_PAPER,this->mateLoadStatus);

	this->setButtonST(BUTTON_PAUSE,false);
	this->setCtrlLedAction(BUTTON_PAUSE,false);

	this->setButtonST(BUTTON_START,false);
	this->setCtrlLedAction(BUTTON_START,false);

}

// 获得取按钮状态
bool SJCutter::getButtonST(u_int key){
	return this->buttonSet[key][1]>0;
}

// 设置LED
void SJCutter::setCtrlLedAction(int key,int action){
	// 改变LED活动状态
	this->ledSet[key][1]=action;
	// 立即响应开关机动作，其它动作需要等待轮询
	if(this->ledSet[key][1]==LEDActions::LED_OFF){
		digitalWrite(key, LOW);
	}else if(this->ledSet[key][1]==LEDActions::LED_ON){
		digitalWrite(key, HIGH);
	}
}

// 数据解码
bool SJCutter::decodeData(uint8_t client,char* line){ 

	// char* temp = (char*)malloc(strlen(line));
	if(line[0] != '^')
	{
	line = strstr(line,(const char*)"^");
		// memcpy(temp,line+1,strlen(line)-1);
		// line = temp;
		
	}

	// 有效数据偏移
	int vdata_offset=5;
	// 编码数据长度
	int data_len=strlen(line); 
	// 指令数据长度
	int gcode_len=data_len-vdata_offset;   
	// 存放行号
	char line_num_data[vdata_offset]; 
	// 存放指令数据
	char gcode_data[gcode_len+1];

	// 设置初始数据
	memset(line_num_data,0,vdata_offset); 
	memcpy(line_num_data,line+1,4);  

	memcpy(gcode_data,line+vdata_offset,gcode_len);   
	// 解码行号
	u_int line_num= base26_decode_int(line_num_data);  
	// 判断接收数据长度是否与原数据长度一致
	bool check_status= (this->taskRecvLine>=line_num);  

	//  grbl_msg_sendf(CLIENT_SERIAL,MsgLevel::Info,"Lost Line:%d",this->taskRecvLine);
	//判断是否丢包
	if(!check_status){ 
		if(this->taskRecvLine>0){
			memset(line_num_data,0,vdata_offset);
			char line_num_data[vdata_offset];
			base26_encode_int(this->taskRecvLine,line_num_data);
			grbl_msg_sendf(client,MsgLevel::Info,"Program Lost:%s",line_num_data);
			grbl_msg_sendf(CLIENT_SERIAL,MsgLevel::Info,"Program Lost:%s",line_num_data);
			// grbl_sendf(CLIENT_ALL, "%cLOST_LINE:%s\r\n", IO_SECURITY_ALERT_LOST_PKG_MARK,line_num_data); 
			// grbl_sendf(CLIENT_SERIAL, "LOST_X:%d,%s,%d,%s,%s\r\n", (this->taskRecvLine-1),line_num_data,line_num,line_num_data2,line); 
		}
	}else{
		// 置换命令数据
		memset(line,0,data_len);
		memcpy(line,gcode_data,gcode_len);  
		this->taskRecvLine=(line_num+1); 
	}  
	// free(temp);
	return check_status;
}

	// 数据编码
CMDResult SJCutter::securityProcess(uint8_t client,char* line){
	// 处理加密码响应
	if (!strncmp(line, ">",1)) {    
		//加密传输请求处理
		uint8_t mark_bit=(uint8_t)line[0];    
		// 判断首字节标记为加密传输请求
		if(mark_bit==IO_SECURITY_REQUEST_MARK){  
			// 计算安全和值
			u_int sval1_bit=(u_int)line[6];
			u_int sval2_bit=(u_int)line[16];

			if(IO_SECURITY_SUMVAL!=(sval1_bit+sval2_bit)){
			// 安全和值不符合预期将中止消息流
			return CMDResult::RESULT_ABANDON;
			} 
			// 准备响应数据包
			uint8_t rsppkg[IO_SECURITY_PKG_SIZE];
			memset(rsppkg,0,IO_SECURITY_PKG_SIZE);
			// 填充随机值
			for(int i=1;i<IO_SECURITY_PKG_SIZE;i++){
			rsppkg[i]=random(65,90);//随机产生可见字符
			}
			rsppkg[0]=IO_SECURITY_RESPONSE_MARK;
			// 产生安全密钥
			rsppkg[6]=random(65,90);
			rsppkg[16]=(127-rsppkg[6]);
			rsppkg[19]=10;
			rsppkg[20]=13;      
			rsppkg[21]=0;      
			// 安全码，用于解码加密包
			this->taskSecurityCode=(rsppkg[6]+rsppkg[16]);
			this->taskRecvLine=0;   
			// 发送响应数据包
			grbl_send(client,(char*)rsppkg);   
			return CMDResult::RESULT_ABANDON;
		}    
	}
	// 处理数据解码
	// grbl_sendf(CLIENT_SERIAL, "client=%d,line=%s\n",client,line);
	// char* temp = (char*)malloc(strlen(line));
	// line = strstr(line, (const char*)"^");
	if (strstr(line, (const char*)"^")) {    
		// 消息数据解码     
		if(!this->decodeData(client,line)){ 
			report_status_message(Error::Ok, client); 
			return CMDResult::RESULT_ABANDON;
		}else{ 
			// report_status_message(Error::Ok, client); 
			// grbl_sendf(CLIENT_SERIAL, "GCODE:%s\r\n",line);  
		}
	} 
	return CMDResult::RESULT_IGNORE;
}

void SJCutter::readSerialNumber(uint8_t client) {
	//读取序列号
	String product_name = sj_product_name->get();  
	String serial_number = sj_serial_number->get();
	grbl_sendf(client, "SN:%s-%s\r\n",product_name.c_str(),serial_number.c_str());    
}

void SJCutter::readFirmwareVersion(uint8_t client) {
	//读取固件版本 
	String firmware_version = FIRMWARE_VERSION;
	grbl_sendf(client, "FV:%s\r\n", firmware_version);    
}

bool SJCutter::writeStepsPerMM(uint8_t client,char* line) {
	//读取默认轴步长参数   
	char val_str[20]; 
	float zero=0.00001;
	float X_Test_Unit=172.0;//横向长度
	float X_offset = sj_cfg_x_offset_11[0]->get();// 默认取第一个为初始值
	float Y_Test_Unit=260.3;//纵向长度
	float Y_offset= sj_cfg_y_offset_11[0]->get();
	float Z_offset= sj_cfg_z_offset->get();
	float X_compens= sj_cfg_x_compens_11[0]->get();
	float Y_compens= sj_cfg_y_compens_11[0]->get(); 

	float X_offset_t=sj_cfg_x_offset_11[0]->get();
	float Y_offset_t=sj_cfg_y_offset_11[0]->get(); 
	float X_compens_t=sj_cfg_x_compens_11[0]->get();
	float Y_compens_t=sj_cfg_y_compens_11[0]->get(); 
	float param_reset=-1.0; //-1.0：初始状态; 0.0：重置参数; 1.0：不重置参数。
	int param_index=0;// 默认取第一个为初始值
	
	
	// grbl_sendf(CLIENT_ALL, "line2: %s\r\n",line);
	// 从参数中获取参数：x偏移原+(C+D)/2, x缩放0, y偏移原+(A+B)/2, y缩放0, z偏移0 , 新x偏移原+C, x缩放D-C, y偏移原+A, y缩放B-A, 参数重置，参数索引
	sscanf(line + 5, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d", &X_offset, &X_compens, &Y_offset, &Y_compens, &Z_offset, &X_offset_t, &X_compens_t, &Y_offset_t, &Y_compens_t, &param_reset, &param_index);
	// grbl_sendf(CLIENT_ALL, "CMVout: %f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d\r\n", X_offset, X_compens, Y_offset, Y_compens, Z_offset, X_offset_t, X_compens_t, Y_offset_t, Y_compens_t, param_reset, param_index);


	// // 从参数中获取参数 
	// sscanf(line + 5, "%lf,%lf,%lf,%lf,%lf,%d", &X_offset, &X_compens, &Y_offset, &Y_compens, &Z_offset, &param_index);

	float old_x_steps_per_mm=axis_settings[X_AXIS]->steps_per_mm->get();
	float old_y_steps_per_mm=axis_settings[Y_AXIS]->steps_per_mm->get();

	
	if (-1.0 == param_reset)
	{
		// 当代入值为0时重置为默认单位步数
		if((X_offset>-zero&&X_offset<zero)&&(X_compens>-zero&&X_compens<zero)){
			old_x_steps_per_mm=DEFAULT_X_STEPS_PER_MM;
		}

		// 当代入值为0时重置为默认单位步数
		if((Y_offset>-zero&&Y_offset<zero)&&(Y_compens>-zero&&Y_compens<zero)){
			old_y_steps_per_mm=DEFAULT_Y_STEPS_PER_MM; 
		}
	}else if (0.0 == param_reset)
	{
		old_x_steps_per_mm=DEFAULT_X_STEPS_PER_MM;
		old_y_steps_per_mm=DEFAULT_Y_STEPS_PER_MM; 
	}

	// 写入X偏移配置
	if (-1.0 == param_reset)
	{
		memset(val_str,0, 20);
		sprintf(val_str, "%0.2f", X_offset);
		sj_cfg_x_offset_11[param_index]->setStringValue(val_str);
	}else if (1.0 == param_reset)
	{
		memset(val_str,0, 20);
		sprintf(val_str, "%0.2f", X_offset_t);
		sj_cfg_x_offset_11[param_index]->setStringValue(val_str);
	}else if (0.0 == param_reset)
	{
		memset(val_str,0, 20);
		sprintf(val_str, "%0.2f", DEFAULT_X_OFFSET_CALIBRATE);
		sj_cfg_x_offset_11[param_index]->setStringValue(val_str);
	}

	// 写入X补偿配置
	if (-1.0 == param_reset)
	{
		memset(val_str,0, 20);
		sprintf(val_str, "%0.5f", DEFAULT_X_COMPENS);
		sj_cfg_x_compens_11[param_index]->setStringValue(val_str);
	}else if (1.0 == param_reset)
	{
		//Scale_test 25/09/05 X缩放补偿配置
		float x_finally_compens = (( sj_cfg_x_compens_11[param_index]->get() != 0.0) ?  sj_cfg_x_compens_11[param_index]->get() : 1.0);
		float X_scale = X_Test_Unit / (((X_Test_Unit - X_compens_t) != 0.0) ? (X_Test_Unit - X_compens_t) : 0.1);
		if(X_scale >= 1.1 || X_scale <= 0.9){
			//缩放校准值异常，保留原有缩放补偿值
			memset(val_str,0, 20);
			sprintf(val_str, "%0.5f", x_finally_compens);
			sj_cfg_x_compens_11[param_index]->setStringValue(val_str);
		}else{
			float x_finally_compens_t = x_finally_compens * X_scale;
			if(x_finally_compens_t > 0.9 && x_finally_compens_t < 1.1){
				memset(val_str,0, 20);
				sprintf(val_str, "%0.5f", x_finally_compens_t);
				sj_cfg_x_compens_11[param_index]->setStringValue(val_str);
			}else{
				//缩放校准计算结果异常，保留原有缩放补偿值
				memset(val_str,0, 20);
				sprintf(val_str, "%0.5f", x_finally_compens);
				sj_cfg_x_compens_11[param_index]->setStringValue(val_str);
			}
		}
	}else if (0.0 == param_reset)
	{
		memset(val_str,0, 20);
		sprintf(val_str, "%0.5f", DEFAULT_X_COMPENS);
		sj_cfg_x_compens_11[param_index]->setStringValue(val_str);
	}

	// 计算X轴单位步数并写入X轴单位步数配置 
	// float x_steps_per_mm=(old_x_steps_per_mm*X_Test_Unit)/(X_Test_Unit+X_compens); 
	float x_steps_per_mm = (old_x_steps_per_mm*X_Test_Unit)/(X_Test_Unit); 
	memset(val_str,0, 20);
	sprintf(val_str, "%0.2f", x_steps_per_mm);
	axis_settings[X_AXIS]->steps_per_mm->setStringValue(val_str);

	// 写入Y偏移配置
	if (-1.0 == param_reset)
	{
		memset(val_str,0, 20);
		sprintf(val_str, "%0.2f", Y_offset);
		sj_cfg_y_offset_11[param_index]->setStringValue(val_str);
	}else if (1.0 == param_reset)
	{
		memset(val_str,0, 20);
		sprintf(val_str, "%0.2f", Y_offset_t);
		sj_cfg_y_offset_11[param_index]->setStringValue(val_str);
	}else if (0.0 == param_reset)
	{
		memset(val_str,0, 20);
		sprintf(val_str, "%0.2f", DEFAULT_Y_OFFSET_CALIBRATE);
		sj_cfg_y_offset_11[param_index]->setStringValue(val_str);
	}

	// 写入Y补偿配置
	if (-1.0 == param_reset)
	{
		memset(val_str,0, 20);
		sprintf(val_str, "%0.5f", DEFAULT_Y_COMPENS);
		sj_cfg_y_compens_11[param_index]->setStringValue(val_str);
	}else if (1.0 == param_reset)
	{
		//Scale_test 25/09/05 Y缩放补偿配置
		float y_finally_compens = ((sj_cfg_y_compens_11[param_index]->get() != 0.0) ? sj_cfg_y_compens_11[param_index]->get() : 1.0);
		float Y_scale = Y_Test_Unit / (((Y_Test_Unit - Y_compens_t) != 0.0) ? (Y_Test_Unit - Y_compens_t) : 0.1);
		if(Y_scale >= 1.1 || Y_scale <= 0.9){
		//缩放校准值异常，保留原有缩放补偿值
			memset(val_str,0, 20);
			sprintf(val_str, "%0.5f", y_finally_compens);
			sj_cfg_y_compens_11[param_index]->setStringValue(val_str);
		}else{
			float y_finally_compens_t = y_finally_compens * Y_scale;
			if(y_finally_compens_t > 0.9 && y_finally_compens_t < 1.1){
				memset(val_str,0, 20);
				sprintf(val_str, "%0.5f", y_finally_compens_t);
				sj_cfg_y_compens_11[param_index]->setStringValue(val_str);
			}else{
				//缩放校准计算结果异常，保留原有缩放补偿值
				memset(val_str,0, 20);
				sprintf(val_str, "%0.5f", y_finally_compens);
				sj_cfg_y_compens_11[param_index]->setStringValue(val_str);
			}
		}
	}else if (0.0 == param_reset)
	{
		memset(val_str,0, 20);
		sprintf(val_str, "%0.5f", DEFAULT_Y_COMPENS);
		sj_cfg_y_compens_11[param_index]->setStringValue(val_str);
	}

	// 计算Y轴单位步数并写入Y轴单位步数配置
	// float y_steps_per_mm=((old_y_steps_per_mm*Y_Test_Unit)/(Y_Test_Unit+Y_compens));
	float y_steps_per_mm = ((old_y_steps_per_mm*Y_Test_Unit)/(Y_Test_Unit));
	memset(val_str,0, 20);
	sprintf(val_str, "%0.2f", y_steps_per_mm);
	axis_settings[Y_AXIS]->steps_per_mm->setStringValue(val_str);

	// // 写入Z轴偏移配置
	// memset(val_str,0, 20);
	// sprintf(val_str, "%0.2f", Z_offset); 
	// sj_cfg_z_offset->setStringValue(val_str);

	// grbl_sendf(client, "CMV:%0.2f,%0.2f,%0.2f\r\n", X_steps_per_mm,Y_steps_per_mm,Z_steps_per_mm);  
	return true;   
}

void SJCutter::readStepsPerMM(uint8_t client) {

	for(int i=0;i<11;i++){
		int param_index = 10 - i;// 校准参数总数11组，按照倒序发送
		// grbl_sendf(CLIENT_SERIAL,"%d\n",param_index);
		float z_offset=  sj_cfg_z_offset->get();
		float x_offset=  sj_cfg_x_offset_11[param_index]->get();
		float y_offset=  sj_cfg_y_offset_11[param_index]->get();
		float x_compens=  sj_cfg_x_compens_11[param_index]->get();
		float y_compens=  sj_cfg_y_compens_11[param_index]->get(); 
		grbl_sendf(client, "CMV:%0.2f,%0.5f,%0.2f,%0.5f,%0.2f,%d\r\n", x_offset,x_compens,y_offset,y_compens,z_offset,param_index);  
	}
}

bool SJCutter::writeZOffset(uint8_t client,char* line) {
	//读取轴步长参数   
	char val_str[20]; 
	float zero=0.00001;
	double Z_offset=0.0;     

	// 从参数中获取参数 
	sscanf(line + 5, "%lf", &Z_offset);

	// 写入Z轴偏移配置
	memset(val_str,0, 20);
	sprintf(val_str, "%0.2f", Z_offset); 
	sj_cfg_z_offset->setStringValue(val_str);

	return true;   
}

void SJCutter::adjustZValue(bool isG0, char* line) {
	float z_delta=  sj_cfg_z_offset->get();
	char* z_pos = strchr(line, 'Z');
	if (z_pos == NULL) {
		// Z值不存在，无法调整
		return;
	}

	char gcmd[20];
	memset(gcmd,0,sizeof(gcmd));
	memcpy(gcmd,line,z_pos-line); 

	float z_val = atof(z_pos + 1);
	if(isG0)
		ZCommand = z_val;
	
	if(!(z_val>-1&&z_val<1)){
		if(z_val<0){
		z_val += (-(z_delta));
		}else if(z_val>0){
		z_val += z_delta;
		} 
		char new_z_val[25];
		memset(new_z_val,0, 25); 
		sprintf(new_z_val,"%sZ%0.2f",gcmd,z_val);
		memcpy(line,new_z_val,25);  
	}   
}


void SJCutter::modifyCommendLine(char* line) {
	//PA->Z-; PB->Z+
	char* ptr = strrchr(line, 'P');
	if (ptr) {
		if (ptr[1] == 'A') {
			ptr[0] = 'Z';
			ptr[1] = '-';
		} else if (ptr[1] == 'B') {
			ptr[0] = ' ';
			ptr[1] = 'Z';
		}
	}
}

void SJCutter::readStatus(char* status_info,uint8_t status_tag) { 
	u_int button_1=this->getButtonST(BUTTON_POWER)?1:0;
	u_int button_2=this->getButtonST(BUTTON_PAUSE)?1:0;
	u_int button_3=this->getButtonST(BUTTON_PAPER)?1:0;
	u_int button_4=this->getButtonST(BUTTON_START)?1:0;
	int sensor_1=digitalRead(X_LIMIT_PIN);
	int sensor_2=digitalRead(PEN1_END_STOP);
	int sensor_3=digitalRead(PEN2_END_STOP);
	int sensor_4=digitalRead(PIN_LED_PAPER);
	int sensor_5=analogRead(PIN_SEEK_BOX);

	u_int readyStatus=this->readyStatus;
	u_int mateLoadStatus=this->mateLoadStatus;
	u_int taskRunStatus=this->taskRunStatus;
	u_int taskPausedStatus=this->taskPausedStatus;
	float* axis_position = system_get_mpos();
	mpos_to_wpos(axis_position);
	float unit_conv = 1.0;      // unit conversion multiplier..default is mm
	if (report_inches->get()) {
		unit_conv = 1.0 / MM_PER_INCH;
	}
	float x_axis=axis_position[X_AXIS];
	float y_axis=axis_position[Y_AXIS];
	float z_axis=axis_position[Z_AXIS];
	// char * state_info=report_state_text();
	// sprintf(status_info, "SI %s|X %4.3f|Y %4.3f|Z %4.3f|P %u%u%u%u|SV %u%u%u%u|LV %u\r\n",state_info, x_axis*unit_conv,y_axis*unit_conv,z_axis*unit_conv,button_1,button_2,button_3,button_4,sensor_1,sensor_2,sensor_3,sensor_4,sensor_5);
	// int tValue = analogRead(PIN_MENU_KEY);
	// |DV:%u%u%u%u ,readyStatus,taskRunStatus,mateLoadStatus,taskPausedStatus
	if(status_tag==0){
		sprintf(status_info, "BV:%u%u%u%u|SV:%u%u%u%u|LV:%u|MKV:%u",button_1,button_2,button_3,button_4,sensor_1,sensor_2,sensor_3,sensor_4,sensor_5,last_btn_val);
	}else if(status_tag==1){
		sprintf(status_info, "BV:%u%u%u%u|SV:%u%u%u%u|LV:%u|MKV:%u|X:%4.3f|Y:%4.3f|Z:%4.3f",button_1,button_2,button_3,button_4,sensor_1,sensor_2,sensor_3,sensor_4,sensor_5,last_btn_val, x_axis*unit_conv,y_axis*unit_conv,z_axis*unit_conv);
	}
	last_btn_val=5000;
}

void SJCutter::exchangeMateLoadStatus(uint8_t client) {
	// 任务运行状态操作无效  
	if(this->taskRunStatus){
		return;
	}
	// 变更加载状态  
	if(this->mateLoadStatus){    
		this->mateLoadStatus=false;    
		SJMotorCtrl::unLoadMate(client);
	}else{
		this->mateLoadStatus=true;
		SJMotorCtrl::loadMate(client);
	}
	// 变更LED状态 
	this->setCtrlLedAction(BUTTON_PAPER,this->mateLoadStatus?LEDActions::LED_ON:LEDActions::LED_OFF);
	// 变更按钮状态 
	this->setButtonST(BUTTON_PAPER,this->mateLoadStatus);
	// 立即报告状态
	report_realtime_status(client); 
}

void SJCutter::exchangeTaskPausedStatus(uint8_t client) {   
	// 非任务运行状态操作无效  
	if(!this->taskRunStatus){
		return;
	}
	// 变更暂停状态  
	if(this->taskPausedStatus){
		this->taskPausedStatus=false; 
	}else{
		this->taskPausedStatus=true; 
	}
	// 变更LED状态 
	this->setCtrlLedAction(BUTTON_PAUSE,this->taskPausedStatus?LEDActions::LED_ON:LEDActions::LED_OFF);
	// 变更按钮状态 
	this->setButtonST(BUTTON_PAUSE,this->taskPausedStatus);  
	// 立即报告状态
	report_realtime_status(CLIENT_ALL); 
}

void SJCutter::TaskClear(uint8_t client) {
	if(this->taskRunStatus){
		SJMotorCtrl::executeCommand(client,"G0Z0F5000");
		char cmdLine[30];
		memset(cmdLine,0,30);
		// X轴机械结构拉出距离
		float x_pulloff_dist = sj_cfg_x_pulloff_dist->get();
		sprintf(cmdLine, "G0 X%0.2fY0",x_pulloff_dist);
		SJMotorCtrl::executeCommand(client,cmdLine);
		protocol_buffer_synchronize();
		memset(cmdLine,0,30);
		float y_offset = DEFAULT_Y_OFFSET;
		sprintf(cmdLine, "G0 Y%.2f",y_offset);
		SJMotorCtrl::executeCommand(client,cmdLine);
		// SJMotorCtrl::executeCommand(client,"G92.1");
		// protocol_buffer_synchronize();
		SJMotorCtrl::executeCommand(client,"M2");
		protocol_buffer_synchronize();
	}

	// 任务清除
	this->taskPausedStatus=false;  
	this->taskRunStatus=false;  
	this->taskSecurityCode=0;
	this->taskRecvLine=0;
	this->resetButtonST();
	grbl_msg_sendf(client,MsgLevel::Info,"Program Clear");
}

void SJCutter::writeRangeCompen(char* line) {	
	// 写入补偿参数 
  	sj_compen_point_set->setStringValue(line + 6);
}

void SJCutter::calcuRangeCompen() {	
	// 获取参数 
	char* line = (char*)sj_compen_point_set->get();
  	sscanf(line, "%lf:%lf,%lf:%lf,%lf:%lf,%lf:%lf,%lf:%lf", &range[0][0], &range[0][1], &range[1][0], &range[1][1], &range[2][0], &range[2][1], &range[3][0], &range[3][1], &range[4][0], &range[4][1]);
	for(uint8_t i = 0; i < 5; i++)
	{
		if(range[i][1] >= 10.0)
			range[i][1] = 10.0;
		if(range[i][1] <= -10.0)
			range[i][1] = -10.0;
	}
	for(uint8_t i = 0; i < 4; i++)
	{
		if(range[i+1][0] - range[i][0] != 0)
		{
			k[i] = (range[i+1][1] - range[i][1]) / (range[i+1][0] - range[i][0]);	//一共5个点，也就是四段区间，计算四条直线斜率，存入k数组中
		}
	}
}

float SJCutter::returnCompen(float x) {
	float compen = 0.0;
	if(x <= range[1][0])
		compen = k[0] * (x - range[0][0]) + range[0][1];	//通过斜率构建一次函数表达式
	else if(x > range[1][0] && x<= range[2][0])
		compen = k[1] * (x - range[1][0]) + range[1][1];
	else if(x > range[2][0] && x<= range[3][0])
		compen = k[2] * (x - range[2][0]) + range[2][1];
	else if(x > range[3][0])
		compen = k[3] * (x - range[3][0]) + range[3][1];
	return compen;
}