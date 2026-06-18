// Copyright (c) 2022 -	Neil
#include "../../Grbl.h"
#include "../SJMotorCtrl.h"
#include "../Common.h"
#include "AccelStepper.h"
#include "MultiStepper.h"
#include "FS.h"
#include "SPIFFS.h"
#include "../../SettingsDefinitions.h"

// extern portMUX_TYPE myMutex;
extern parser_state_t gc_state;
extern parser_block_t gc_block;

extern FloatSetting* sj_cfg_l_chn_x_offset;
extern FloatSetting* sj_cfg_l_chn_Y_offset;
extern FloatSetting* sj_cfg_r_chn_X_offset;
extern FloatSetting* sj_cfg_r_chn_y_offset;
extern FloatSetting* sj_cfg_sps_x_offset;
extern FloatSetting* sj_cfg_sps_y_offset;

int   lastDirX       = 0;
int   lastDirY       = 0;
long  laserSteps     = 0;
float posY           = 0;
bool  lasering       = false;
float distance       = 0;
float loadMateLength = 50;  //加载物料的长度

float spacer         = 0.075;  //0.064;
float backDistance   = 0.3;
float bankDistance   = 2;
float accleDistance  = 12;
int   g_MaxSpeed     = 3000;  //90000;
int   g_MaxAccel     = 5000;  //120000;
int   g_SafeMode     = 0;
bool  g_PauseAndStop = 0;
int  z_homing_try_count = 0;


long toStepX(float mm) {
    float steps_per_mm = axis_settings[X_AXIS]->steps_per_mm->get();
    return steps_per_mm * mm;
}

float toMMX(long vSteps) {
    float steps_per_mm = axis_settings[X_AXIS]->steps_per_mm->get();
    return vSteps / steps_per_mm;
}

long toStepY(float mm) {
    float steps_per_mm = axis_settings[Y_AXIS]->steps_per_mm->get();
    return steps_per_mm * mm;
}

float toMMY(long vSteps) {
    float steps_per_mm = axis_settings[Y_AXIS]->steps_per_mm->get();
    return vSteps / steps_per_mm;
}

long toStepZ(float mm) {
    float steps_per_mm = axis_settings[Z_AXIS]->steps_per_mm->get();
    return steps_per_mm * mm;
}

float toMMZ(long vSteps) {
    float steps_per_mm = axis_settings[Z_AXIS]->steps_per_mm->get();
    return vSteps / steps_per_mm;
}

int dirs = 0;

void forwardX(void) {
    dirs &= ~(bit(X_AXIS));

    motors_direction(dirs);
    motors_step(bit(X_AXIS));
}

void backwardX(void) {
    dirs |= bit(X_AXIS);

    motors_direction(dirs);
    motors_step(bit(X_AXIS));
}

void forwardY(void) {
    dirs &= ~(bit(Y_AXIS));

    motors_direction(dirs);
    motors_step(bit(Y_AXIS));
}

void backwardY(void) {
    dirs |= bit(Y_AXIS);

    motors_direction(dirs);
    motors_step(bit(Y_AXIS));
}

void forwardZ(void) {
    dirs |= bit(Z_AXIS);

    motors_direction(dirs);
    motors_step(bit(Z_AXIS));
}

void backwardZ(void) {
    dirs &= ~(bit(Z_AXIS));

    motors_direction(dirs);
    motors_step(bit(Z_AXIS));
}
//////////////////////////////////////////////////////////////////
AccelStepper motorX(forwardX, backwardX);
AccelStepper motorY(forwardY, backwardY);
AccelStepper motorZ(forwardZ, backwardZ);

//////////////////////////////////////////////////////////////////

void SJMotorCtrl::init() {
    motorX.setAcceleration(1000);
    motorX.setSpeed(0);
    motorX.setCurrentPosition(0);
    motorX.setMaxSpeed(3000);

    motorY.setAcceleration(1000);
    motorY.setSpeed(0);
    motorY.setCurrentPosition(0);
    motorY.setMaxSpeed(3000);

    motorZ.setAcceleration(toStepZ(2000));
    motorZ.setSpeed(0);
    motorZ.setCurrentPosition(0);
    motorZ.setMaxSpeed(2000);
}

void SJMotorCtrl::loop() {}

void SJMotorCtrl::initZAxis() {
    motorZ.setAcceleration(toStepZ(2000));
    motorZ.setSpeed(0);
    motorZ.setMaxSpeed(2000);
}

bool SJMotorCtrl::waitRunStatus() {
    bool result=true;
    while (true)
    {
        if((sys.state!=State::Homing&&sys.state!=State::Jog)){
            break;
        // // }        
        // if((sys.state==State::Idle)){&&sys.state!=State::Cycle
        //     break;
        }else{
            yield();
        }
    }
    return result;
}

bool SJMotorCtrl::yAxisHoming(uint8_t client) {
    //初始Y轴
    motorY.setAcceleration(toStepY(3000));
    motorY.setSpeed(0);
    motorY.setMaxSpeed(toStepY(3000 / 60)); 
    motorY.runToNewPosition(-(motorY.currentPosition()));
 
}
bool SJMotorCtrl::zAxisHoming1(uint8_t client) {

    // 等待可用状态
    if(!SJMotorCtrl::waitRunStatus()){
        SJMotorCtrl::initZAxis();
        return false;
    }

    // 判断尝试次数
    if(z_homing_try_count>5){
        z_homing_try_count=0;
        grbl_sendf(CLIENT_ALL, "err:pen check 1\r\n");
        SJMotorCtrl::initZAxis();
        return false;
    }

    //下笔最深5mm
    int try_distance=2000;
    sys.state = State::Homing;
    bool limit_pos1_tag = false;
    

    motors_unstep();
    motors_set_disable(false);

    //motorZ.setAcceleration(toStepZ(2000));
    //motorZ.setSpeed(0);
    //motorZ.setMaxSpeed(2000);

    // float z_steps_acceleration = axis_settings[Z_AXIS]->acceleration->get();
    // float z_steps_max_rate = axis_settings[Z_AXIS]->max_rate->get();
    motorZ.setAcceleration(toStepZ(500));
    motorZ.setSpeed(9000);
    motorZ.setMaxSpeed(9000);

    
        // grbl_sendf(CLIENT_SERIAL, "CHUFA\n");
    while(z_homing_try_count<5)
    {
        // grbl_sendf(CLIENT_SERIAL, "CHUFA\n");
        if (digitalRead(PEN1_END_STOP) == 1)
            break;
        //碰到第一个限位
        motorZ.moveTo(toStepZ(-try_distance));
        while (motorZ.distanceToGo())
        {
            if (digitalRead(PEN1_END_STOP) == 1){
                z_homing_try_count = 0;
                break;
            }
            motorZ.run();
            yield();
        }


        if (motorZ.distanceToGo() == 0)
        {
            z_homing_try_count++;
            // motorZ.runToNewPosition(toStepZ(toMMZ(motorZ.currentPosition())));
            continue;
        }
        if(!limit_pos1_tag){
            limit_pos1_tag=true;
            motorZ.setCurrentPosition(0);
            // motorZ.runToNewPosition(toStepZ(toMMZ(motorZ.currentPosition())+lacuna));
        }else{
            float limit_pos2=toMMZ(motorZ.currentPosition());
            grbl_sendf(CLIENT_ALL, "pen home ok bank %0.2f\r\n", limit_pos2);
            break;
        }


        // if(!limit_pos1_tag)
        // {
        //     limit_pos1_tag=true;
        //     motorZ.setCurrentPosition(0);
        //     motorZ.runToNewPosition(toStepZ(toMMZ(motorZ.currentPosition())+lacuna));
        // }
        // else
        // {
        //     float limit_pos2=toMMZ(motorZ.currentPosition());

        //     if((limit_pos2 > 30) && (limit_pos2 < 40))
        //     {
        //         motorZ.runToNewPosition(0);
        //         break;
        //     }
        //     else
        //     {
        //         limit_pos1_tag=false;
        //     }


        // }
        z_homing_try_count++;
    }
    motorZ.setCurrentPosition(0);
    gc_block.values.xyz[Z_AXIS] = 0;
    gc_state.coord_offset[Z_AXIS] = 0;
    // memcpy(gc_state.coord_offset, gc_block.values.xyz, sizeof(gc_block.values.xyz));
    system_flag_wco_change();
    sys.state = State::Idle;
    SJMotorCtrl::executeCommand(CLIENT_SERIAL,"$HZ");
    protocol_buffer_synchronize();
    if(sys.state == State::Alarm)
        SJCutterIns.zHomeStatus = false;
    return true;
}

bool SJMotorCtrl::zAxisHoming(uint8_t client) {

    // 等待可用状态
    if(!SJMotorCtrl::waitRunStatus()){
        SJMotorCtrl::initZAxis();
        return false;
    }

    // 判断尝试次数
    if(z_homing_try_count>5){
        z_homing_try_count=0;
        grbl_sendf(CLIENT_ALL, "err:pen check 1\r\n");
        SJMotorCtrl::initZAxis();
        return false;
    }
    
    //下笔最深5mm
    int try_distance=20;
    sys.state = State::Homing;
#if defined(PEN1_END_STOP) && defined(PEN2_END_STOP)
    //假设电机负方向是往左转
    

    //如果两个限位都没有碰到，属于异常情况
    if (digitalRead(PEN1_END_STOP) == LIMIT_TOUCH && digitalRead(PEN2_END_STOP) == LIMIT_TOUCH) {
        grbl_sendf(CLIENT_ALL, "err:pen check 0\r\n");
        sys.state = State::Idle;  //Alarm
        z_homing_try_count++;
        SJMotorCtrl::initZAxis();

        return SJMotorCtrl::zAxisHoming(client);
    }

    motors_unstep();
    motors_set_disable(false); 
    motorZ.setAcceleration(toStepZ(1000)); 
    motorZ.setSpeed(0);
    motorZ.setMaxSpeed(toStepZ(1000 / 60)); 

    //如果左边限位没有碰到，电机往左转，抬起右边压轮
    motorZ.moveTo(toStepZ(-try_distance));
    while (motorZ.distanceToGo()) {
        if (digitalRead(PEN1_END_STOP) == LIMIT_TOUCH)
            break;

        motorZ.run();
        yield();
    }

    //如果转动完，左边限位还没碰到，属于异常情况
    if (motorZ.distanceToGo() == 0) {
        motors_unstep();
        motors_set_disable(true);
        // grbl_sendf(CLIENT_ALL, "err:pen check 1\r\n");
        sys.state = State::Idle;  //Alarm
        z_homing_try_count++;

        SJMotorCtrl::initZAxis();        
        return SJMotorCtrl::zAxisHoming(client);
    }

    //如果右边限位没有碰到，电机往右转，抬起左边压轮
    motorZ.moveTo(toStepZ(try_distance));
    while (motorZ.distanceToGo()) {
        if (digitalRead(PEN2_END_STOP) == LIMIT_TOUCH)
            break;

        motorZ.run();
        yield();
    }

    //如果转动完，右边限位还没碰到，属于异常情况
    if (motorZ.distanceToGo() == 0) {
        motors_unstep();
        motors_set_disable(true);
        // grbl_sendf(CLIENT_ALL, "err:pen check 2\r\n");
        sys.state = State::Idle;  //Alarm
        z_homing_try_count++;
        SJMotorCtrl::initZAxis();        
        return SJMotorCtrl::zAxisHoming(client);
    }

 /////////////////////////测量前准备动作结束//////////////////////////////////
/*
当前的状态处于右刀临界位1\0
*/
    //粗略设置当前坐标为0
    motorZ.setCurrentPosition(0);

    //压左边轮子，使左边笔离开限位开关
    motorZ.moveTo(toStepZ(-try_distance));
    while (motorZ.distanceToGo()) {
        if (digitalRead(PEN1_END_STOP) == LIMIT_TOUCH)
            break;
        motorZ.run();
        yield();
    }

    //测得需要下压的距离
    float tPosLeft = toMMZ(motorZ.currentPosition());

//    motorZ.setCurrentPosition(0);
    //右转压右边轮子，使右边笔离开限位开关
    motorZ.moveTo(toStepZ(try_distance));
    while (motorZ.distanceToGo()) {
        if (digitalRead(PEN2_END_STOP) == LIMIT_TOUCH)
            break;

        motorZ.run();
        yield();
    }


    //测得需要下压的距离
    float tPosRight = toMMZ(motorZ.currentPosition());

    //取得中间点
    float tPosMid = (tPosRight + tPosLeft)/ 2;
    motorZ.moveTo(toStepZ(tPosMid));
    while (motorZ.distanceToGo()) {
        motorZ.run();
        yield();
    }
    // grbl_sendf(CLIENT_ALL, "err:pen %f,%f,%f\r\n",tPosLeft, tPosRight,tPosMid);
 
    motors_unstep();
    motors_set_disable(true);
    motorZ.setCurrentPosition(0); 

    //额外计算左右下压的空行程（数值为正)
    float tBankDistance =abs(tPosRight + tPosLeft) / 2;

    //这时，应该左右限位都碰到
    if (digitalRead(PEN1_END_STOP) == LIMIT_TOUCH || digitalRead(PEN2_END_STOP) == LIMIT_TOUCH) {        
        // grbl_sendf(CLIENT_ALL, "err:pen check 0\r\n");
        sys.state = State::Idle;  //Alarm
        z_homing_try_count++;
        SJMotorCtrl::initZAxis();
        return SJMotorCtrl::zAxisHoming(client);
    }


    //设置Z坐标为0
    gc_block.values.xyz[Z_AXIS] = 0;
    gc_state.coord_offset[Z_AXIS] =0;
    // memcpy(gc_state.coord_offset, gc_block.values.xyz, sizeof(gc_block.values.xyz));
    system_flag_wco_change();

    // SJMotorCtrl::executeCommand(client,"G10 P0 L20 Z0");
    //额外计算左右下压的空行程（数值为正)
    // float tBankDistance =abs(tPosMid);
    if (tBankDistance<1.5) {
        sys.state = State::Idle;  //Alarm
        z_homing_try_count++;
        SJMotorCtrl::initZAxis();
        return SJMotorCtrl::zAxisHoming(client);
    }

    
    grbl_sendf(CLIENT_ALL, "pen home ok bank %0.2f\r\n", tBankDistance);
    // grbl_msg_sendf(CLIENT_ALL,MsgLevel::Info,"pen home ok bank %0.2f", tBankDistance);
#endif
    sys.state = State::Idle;  //Alarm
    SJMotorCtrl::initZAxis();
    return true;
}

bool SJMotorCtrl::xyAxisHoming(uint8_t client, bool vMiddle) {

    SJMotorCtrl::executeCommand(client,"$HX");

    // 等待可用状态
    if(!SJMotorCtrl::waitRunStatus()){
        return false;
    }


    sys.state = State::Homing;

    SJMotorCtrl::enableMotor();
    motorX.setAcceleration(toStepX(5000));
    motorX.setSpeed(0);
    motorX.runToNewPosition(toStepX(0));
    motorX.setMaxSpeed(toStepX(9000 / 60));

    

    //向右移，直到小车离开限位
    motorX.moveTo(toStepX(30));
    while (motorX.distanceToGo()) {
        if (digitalRead(X_LIMIT_PIN) != LIMIT_TOUCH)
            break;

        motorX.run();
        yield();
    }

    //如果转动完，限位还没离开，属于异常情况
    if (motorX.distanceToGo() == 0) {
        motors_unstep();
        motors_set_disable(true);
        grbl_sendf(CLIENT_ALL, "error:1000\r\n");

        sys.state = State::Idle;  //Alarm
        return false;
    }

    //停下
    motorX.stop();
    motorX.runToPosition();

    //往限位靠近
    motorX.setCurrentPosition(0);
    motorX.moveTo(toStepX(-350));

    //直到碰到限位
    while (motorX.distanceToGo()) {
        if (digitalRead(X_LIMIT_PIN) == LIMIT_TOUCH)
            break;

        motorX.run();
        yield();
    }

    //如果转动完，限位还没碰到，属于异常情况
    if (motorX.distanceToGo() == 0) {
        motors_unstep();
        motors_set_disable(true);
        grbl_sendf(CLIENT_ALL, "error:1001\r\n");
        sys.state = State::Idle;  //Alarm
        return false;
    }

    //远离一点限位
    motorX.setCurrentPosition(0);
    motorX.moveTo(toStepX(3));

    while (motorX.distanceToGo()) {
        motorX.run();
        yield();
    }

    //motors_unstep();
    //motors_set_disable(true);
    motorX.setCurrentPosition(0);
    motorY.setCurrentPosition(0);

    if (vMiddle) {
        // motorX.moveTo(toStepX(150));
        // while (motorX.distanceToGo()) {
        //     motorX.run();
        //     yield();
        // }


        //设置XY坐标为0
        // gc_block.values.xyz[X_AXIS] = -150;
        // gc_block.values.xyz[Y_AXIS] = 0;
    } else {
        // gc_block.values.xyz[X_AXIS] = 0;
        // gc_block.values.xyz[Y_AXIS] = 0;
    }
    // gc_block.values.xyz[X_AXIS] = 0;
    // gc_block.values.xyz[Y_AXIS] = 0;
    // memcpy(gc_state.coord_system, gc_block.values.xyz, sizeof(gc_block.values.xyz));


    // gc_state.position[X_AXIS] = 0;
    // gc_state.position[Y_AXIS] = 0;
    // SJMotorCtrl::executeCommand(client,"G10 P0 L20 X0Y0");
    // SJMotorCtrl::executeCommand(client,"G10 P1 L20 X0Y0");
    //gc_state.coord_system[idx] + gc_state.coord_offset[idx];
    //机械坐标清零
    // gc_state.coord_system[X_AXIS] = 0;
    // gc_state.coord_system[Y_AXIS] = 0;

    // SJMotorCtrl::executeCommand(client,"G10 P0 L20 X0Y0");
 
    system_flag_wco_change();

    grbl_sendf(CLIENT_ALL, "[MSG]HOME OK\r\n");

    sys.state = State::Idle;

    SJMotorCtrl::disableMotor();

    return true;
}

void SJMotorCtrl::enableMotor(void) {
    motors_unstep();
    motors_set_disable(false);
}

void SJMotorCtrl::disableMotor(void) {
    motors_unstep();
    motors_set_disable(true);
}

void SJMotorCtrl::runTest(uint8_t client) {
    // 任务运行状态操作无效
    if (SJCutterIns.taskRunStatus) {
        return;
    }

    SJCutterIns.testMode = true;
    SJMotorCtrl::zAxisHoming(client);

    SJMotorCtrl::resetAxis(CLIENT_ALL,true,true,false);  

    SJMotorCtrl::enableMotor();

    motorX.setAcceleration(toStepX(2000));
    motorX.setSpeed(0);
    float work_coord_x_offset=homing_pulloff->get()+DEFAULT_HOMING_OFFSET;
    float work_coord_y_offset=DEFAULT_Y_OFFSET;

    // motorX.setCurrentPosition(toStepX(work_coord_x_offset));
    motorX.setCurrentPosition(0);
    motorX.setMaxSpeed(toStepX(18000 / 60));

    motorY.setAcceleration(toStepY(2000));
    motorY.setSpeed(0);
    motorY.setCurrentPosition(0);
    motorY.setMaxSpeed(toStepY(18000 / 60));

    // motorZ.setAcceleration(toStepZ(2000));
    // motorZ.setSpeed(0);
    // motorZ.setCurrentPosition(0);
    // motorZ.setMaxSpeed(toStepZ(4000 / 60));

    while (SJCutterIns.testMode) {
        // 中止测试
        int tValue = analogRead(PIN_MENU_KEY);
        if (tValue < 3000) {
            break;
        }

        motorX.run();
        motorY.run();
        // motorZ.run();

        if (motorX.distanceToGo() == 0)
            if (motorX.currentPosition() == 0)
                motorX.moveTo(toStepX(290));
            else
                motorX.moveTo(toStepX(0));

        if (motorY.distanceToGo() == 0)
            if (motorY.currentPosition() == 0)
                motorY.moveTo(toStepY(290));
            else
                motorY.moveTo(toStepY(0));

        // if (motorZ.distanceToGo() == 0)
        //     if (motorZ.currentPosition() < 0)
        //         motorZ.moveTo(toStepZ(25));
        //     else
        //         motorZ.moveTo(toStepZ(-25));

        yield();
    }

    SJMotorCtrl::disableMotor();

    SJCutterIns.testMode = false;

    SJCutterIns.reboot();
 
}
void SJMotorCtrl::runTest_Rectangle(uint8_t client) {
    // 任务运行状态操作无效
    if (SJCutterIns.taskRunStatus) {
        return;
    }

    SJCutterIns.testMode = true;

    //归位函数与GCode执行函数冲突 未找到解决办法 故使用消息处理器
    SJCutterIns.handleMessage(CLIENT_ALL, "HPA");
    
    //等待归位完成
    protocol_buffer_synchronize();
    
    while(SJCutterIns.testMode){
        // 中止测试
        int tValue = analogRead(PIN_MENU_KEY);
        //跳出外循环
        if (tValue < 3000) {
            SJCutterIns.testMode=false;
            break;
        }
        //自定义GCode内容
        // char omg[] = "G90\nG21\nG0X0Y0Z0\nG0Z-0.7\nG0X1.04Y27.07\nG0Z-2.2\nG1X1.04Y27.07\nG1X2.54Y27.07\nG0Z-0.7\nG0X1.04Y27.07\nG0Z-2.2\nG1X1.04Y27.07\nG1X202.79Y27.070\nG1X303.53Y27.07\nG1X202.79Y27.07\nG1X202.79Y142.510\nG1X202.79Y143.31\nG1X202.79Y142.51\nG1X1.04Y142.510\nG1X0.24Y142.51\nG1X1.04Y142.51\nG1X1.04Y27.070\nG1X1.04Y26.27\nG1X1.04Y27.07\nG1X2.54Y27.07";
        // char GCmd[] = "TS 0,0,0\nTF 5000,7000\nG21\nG90\nG0Z-0.70\nG0X68.09Y10.41\nG0Z-22.00\nG1X68.09Y10.41\nG1X68.29Y10.41\nG0Z-0.70\nG0X68.09Y10.41\nG0Z-22.00\nG1X68.09Y10.41\nG1X221.09Y10.41\nG1X221.89Y10.41\nG1X221.09Y11.21\nG1X221.09Y70.03\nG1X221.09Y70.83\nG1X220.29Y70.03\nG1X68.09Y70.03\nG1X67.29Y70.03\nG1X68.09Y69.23\nG1X68.09Y10.41\nG1X68.09Y9.61\nG1X68.89Y10.41\nG1X68.89Y10.41\nG0Z0\nG0X150Y0\nM2\nTE";
        char GCmd1[] = "TS 0,0,0\nTF 10000,10000\nG21\nG90\nG0Z-0.70\nG0X12.44Y37.85\nG0Z-40.00\nG1X53.09Y78.29\nG0Z-0.70\nG0Z-40.00\nG1X93.22Y37.85\nG0Z-0.70\nG0Z-40.00\nG1X133.22Y78.29\nG0Z-0.70\nG0Z-40.00\nG1X173.22Y37.85\nG0Z-0.70\nG0Z-40.00\nG1X213.22Y78.29\nG0Z-0.70\nG0Z-40.00\nG1X253.22Y37.85\nG0Z-0.70\nG0Z-40.00\nG1X293.22Y78.29\nG0Z-0.70\nG0Z-40.00\nG1X253.22Y37.85\nG0Z-0.70\nG0Z-40.00\nG1X213.22Y78.29\nG0Z-0.70\nG0Z-40.00\n\
        G1X173.22Y37.85\nG0Z-0.70\nG0Z-40.00\nG1X133.22Y78.29\nG0Z-0.70\nG0Z-40.00\nG1X93.22Y37.85\nG0Z-0.70\nG0Z-40.00\nG1X53.09Y78.29\nG0Z-0.70\nG0Z-40.00\nG0X12.44Y37.85\nG0Z-0.70\nG0Z-40.00\nG0Z0\nG0X150Y0\nM2\nTE";
        char sep[] = "\n"; //分隔符
        char* line = NULL; //读取到的数据

        //临时存放指针
        char* ptr = NULL;
        line = strtok_r(GCmd1, sep, &ptr); //根据分隔符迭代GCode
        while(line!=NULL&&SJCutterIns.testMode){
            // 中止测试
            // 由于Gcode执行需同步等待电机运行状态，所以在电机运行途中，该方法无法立即响应按钮以退出测试循环
            int tValue = analogRead(PIN_MENU_KEY);
            // 跳出内循环
            if (tValue < 3000) {
                SJCutterIns.testMode=false;
                break;
            } 
            gc_execute_line(line, CLIENT_SERIAL); 
            line = strtok_r(NULL, sep, &ptr); 
            protocol_buffer_synchronize();
            empty_line(CLIENT_ALL);
        }
    }
    SJMotorCtrl::zAxisHoming1(CLIENT_ALL);
    SJMotorCtrl::resetAxis(client,true,true,true);
    SJMotorCtrl::moveYAxis(CLIENT_ALL,-160); 
    SJCutterIns.mateLoadStatus = false;
    // 变更LED状态 
    SJCutterIns.setCtrlLedAction(BUTTON_PAPER,SJCutterIns.mateLoadStatus?LEDActions::LED_ON:LEDActions::LED_OFF);
    // 变更按钮状态 
    SJCutterIns.setButtonST(BUTTON_PAPER,SJCutterIns.mateLoadStatus);
}

bool SJMotorCtrl::loadMate(uint8_t client) {
    // return SJMotorCtrl::absMoveAxis(client, Y_AXIS, 20);
    SJMotorCtrl::moveYAxis(CLIENT_ALL,50);
}

bool SJMotorCtrl::unLoadMate(uint8_t client) {
    // float mate_dist = sj_cfg_y_mate_dist->get();
    // return SJMotorCtrl::absMoveAxis(client, Y_AXIS, mate_dist);
    if(SJCutterIns.smartMaterial == false)
    {
        SJMotorCtrl::moveYAxis(CLIENT_ALL,-50);
    }
    else
    {
        char cmdLine[30];
        memset(cmdLine,0,30);
        sprintf(cmdLine, "G0Y-20");
        SJMotorCtrl::executeCommand(client,cmdLine);
        SJCutterIns.smartMaterial = false;
    }
}
 

bool SJMotorCtrl::resetAxis(uint8_t client,bool homing,bool reset,bool tocenter) {
    // 等待可用状态
    if(!SJMotorCtrl::waitRunStatus()){
        return false;
    } 

    //电机速度
    // int yspeed = y_axis_settings->max_rate->get();
    // int yacceleration = y_axis_settings->acceleration->get();
    // X轴机械结构拉出距离
    float x_pulloff_dist = sj_cfg_x_pulloff_dist->get();  
    float homing_pulloff_dist = homing_pulloff->get();  
    char cmdLine[64];   
    
    if(homing){
        // X轴Homing
        SJMotorCtrl::executeCommand(client,"$HX");
        protocol_buffer_synchronize();
        if(sys.state == State::Alarm)
            SJCutterIns.xHomeStatus = false;
    }else{
//        float work_coord_x_offset=homing_pulloff->get();
//        // float work_coord_x_offset=homing_pulloff->get()+DEFAULT_HOMING_OFFSET+sj_cfg_x_offset->get();
//        // float work_coord_y_offset=sj_cfg_y_offset->get();+sj_cfg_x_offset->get()
//        // sprintf(cmdLine, "G0X%3.2fY%3.2F3000\0",work_coord_x_offset,0);
//        SJMotorCtrl::executeCommand(client,"G0X0Y0F3000\0");
//        protocol_buffer_synchronize();


        float work_coord_x_offset=homing_pulloff->get()+DEFAULT_X_OFFSET;
        // float work_coord_x_offset=homing_pulloff->get()+DEFAULT_HOMING_OFFSET+sj_cfg_x_offset->get();
        float work_coord_y_offset=DEFAULT_Y_OFFSET;
        sprintf(cmdLine, "G0X%3.2fY%3.2F3000\0",work_coord_x_offset,work_coord_y_offset);
        SJMotorCtrl::executeCommand(client,cmdLine);
        protocol_buffer_synchronize();
    }
     
     // 重新定位机械坐标
    if(reset){
        if(SJMotorCtrl::waitRunStatus()){
            // memset(cmdLine,0,64);
            // sprintf(cmdLine, "G10 P0 L20 X%0.2fY0F3000",x_pulloff_dist);        
            // SJMotorCtrl::executeCommand(client,cmdLine);
            SJMotorCtrl::executeCommand(client,"G10 P0 L20 X0Y0Z0"); 
            protocol_execute_realtime();
        }                
    }  

    // 移动到中间
    if(tocenter){
        if(SJMotorCtrl::waitRunStatus()){
            // SJMotorCtrl::executeCommand(CLIENT_ALL,"$J=G90G21X150Y0F8000"); 
            sprintf(cmdLine, "G0X%0.2fY0F5000",x_pulloff_dist);        
            SJMotorCtrl::executeCommand(client,cmdLine);
            protocol_buffer_synchronize();
            
        }
    }
    // 等待运行状态
    SJMotorCtrl::waitRunStatus();
    // 立即报告状态
	report_realtime_status(CLIENT_ALL);
    return true;
   
}

bool SJMotorCtrl::moveAndResetYAxis(uint8_t client,float y_move_dist) {
    char cmdLine[64];

    memset(cmdLine,0,64);
    sprintf(cmdLine, "$J=G90G21Y%3.2fF3000",y_move_dist);
    execute_line(cmdLine, client, WebUI::AuthenticationLevel::LEVEL_GUEST);

    protocol_buffer_synchronize();
      // 重置Y轴0位；
    memset(cmdLine,0,64);
    sprintf(cmdLine, "G10 P0 L20 Y0",0);
    execute_line(cmdLine, client, WebUI::AuthenticationLevel::LEVEL_GUEST);
    protocol_buffer_synchronize();
}

bool SJMotorCtrl::moveYAxis(uint8_t client,float y_move_dist) {
    // 等待可用状态
    if(!SJMotorCtrl::waitRunStatus()){
        return false;
    }

    char cmdLine[64];

    sys.state = State::Homing;  //Alarm
    //电机准备
    motors_unstep();
    motors_set_disable(false); 
    //电机速度
    int yspeed = y_axis_settings->max_rate->get();
    int yacceleration = y_axis_settings->acceleration->get();
    
    // //初始Y轴
    // motorY.setAcceleration(1000);
    // motorY.setSpeed(0);
    // motorY.setMaxSpeed(toStepY(3000));

    motorY.setAcceleration(toStepY(3000));
    motorY.setSpeed(0);
    motorY.setMaxSpeed(toStepY(3000 / 60)); 

    // 回到原位
    // motorY.runToNewPosition(toStepY(y_move_dist));
    motorY.moveTo(toStepY(y_move_dist));
    while (motorY.distanceToGo()) { 
        motorY.run();
        yield();
    }


    motorY.setCurrentPosition(0); 

    //设置Z坐标为0
    gc_block.values.xyz[Y_AXIS] = 0;
    gc_state.coord_offset[Y_AXIS] = 0;
    // memcpy(gc_state.coord_offset, gc_block.values.xyz, sizeof(gc_block.values.xyz));
    system_flag_wco_change();

    //释放电机
    motors_unstep();
    motors_set_disable(true);

    sys.state = State::Idle;  //Alarm
}



bool SJMotorCtrl::absMoveAxis(uint8_t client, uint axis, float moveDist) {
    char jogLine[LINE_BUFFER_SIZE]; 
    sprintf(jogLine, "$J=G90G21%s%3.2fF%d", axis == 0 ? "X" : "Y", moveDist,sj_cfg_mate_load_speed->get());
    gc_execute_line(jogLine, client);
    // grbl_sendf(client, "mateLoadStatus:%s\r\n",jogLine);
    protocol_buffer_synchronize();
    return true;
}

bool SJMotorCtrl::relMoveAxis(uint8_t client, uint axis, float moveDist) {
    char jogLine[LINE_BUFFER_SIZE];
    sprintf(jogLine, "$J=G91G21%s%3.2fF3000", axis == 0 ? "X" : "Y", moveDist);
    // grbl_send(client, jogLine);
    gc_execute_line(jogLine, client);
    protocol_buffer_synchronize();
    return true;
}


void SJMotorCtrl::executeCommand(uint8_t client,const char *cmd) {
    char cmdLine[64];
    // X轴归位
    memset(cmdLine,0,64);
    sprintf(cmdLine, "%s",cmd);
    execute_line(cmdLine, client, WebUI::AuthenticationLevel::LEVEL_USER);     
}
 

void SJMotorCtrl::executeRealtimeCommand(uint8_t client,char cmd) {
    char cmdLine[2];
    // X轴归位
    memset(cmdLine,0,2); 
    memset(cmdLine,cmd,1); 
    execute_line(cmdLine, client, WebUI::AuthenticationLevel::LEVEL_USER);     
}

void SJMotorCtrl::SeekActionStart(int speed) {

     //电机准备
    motors_unstep();
    motors_set_disable(false);
    //初始X轴
    motorX.setAcceleration(toStepX(4000));
    motorX.setSpeed(0);
    motorX.setMaxSpeed(toStepX(speed/60));

    //初始Y轴
    motorY.setAcceleration(toStepY(4000));
    motorY.setSpeed(0);
    motorY.setMaxSpeed(toStepY(speed/60));

    float work_coord_x_offset=homing_pulloff->get()+DEFAULT_HOMING_OFFSET+DEFAULT_X_OFFSET;
    float work_coord_y_offset=DEFAULT_Y_OFFSET;  

    // motorX.setCurrentPosition(toStepX(work_coord_x_offset));
    // motorY.setCurrentPosition(toStepY(work_coord_y_offset));

    // motorX.setCurrentPosition(toStepX(0));
    // motorY.setCurrentPosition(toStepY(0));
    motorX.runToNewPosition(toStepX(0));
    motorY.runToNewPosition(toStepY(0));

}

void SJMotorCtrl::SeekActionEnd() {
    float work_coord_x_offset=homing_pulloff->get()+DEFAULT_HOMING_OFFSET+DEFAULT_X_OFFSET;
    float work_coord_y_offset=DEFAULT_Y_OFFSET;  
    // 回到原位
    motorX.runToNewPosition(toStepX(0));
    motorY.runToNewPosition(toStepY(0));  

    // motorX.setAcceleration(5000);
    // motorX.setSpeed(0);
    // motorX.setMaxSpeed(3000);

    // motorY.setAcceleration(5000);
    // motorY.setSpeed(0);
    // motorY.setMaxSpeed(3000);

    motorX.setAcceleration(DEFAULT_X_ACCELERATION);
    motorX.setSpeed(0);
    motorX.setMaxSpeed(DEFAULT_X_MAX_RATE);

    motorY.setAcceleration(DEFAULT_Y_ACCELERATION);
    motorY.setSpeed(0);
    motorY.setMaxSpeed(DEFAULT_Y_MAX_RATE);

     //释放电机
    motors_unstep();
    motors_set_disable(true);    
    
}


int SJMotorCtrl::SeekPointByXY(float x, float y) {  
    // x坐标最大值
    float maxXAxis = 345; 
#if defined(PIN_SEEK_BOX) 
    // 当超取识别范围直接判断为识别异常
    if (x <= maxXAxis) {
        // 移动电机到指定位置
        motorX.runToNewPosition(toStepX(x));
        motorY.runToNewPosition(toStepY(y));
        //获取寻边传感器值
        return  analogRead(PIN_SEEK_BOX); 
    } else {
        return 0;
    }
#else
    return 0;
#endif
}



 
int SJMotorCtrl::TestFindPointByRange(float maxval,int dirval,int speedval){
 
    //电机准备
    motors_unstep();
    motors_set_disable(false); 

    //初始X轴
    motorX.setAcceleration(toStepX(speedval));
    motorX.setSpeed(0);
    motorX.setMaxSpeed(toStepX(speedval / 60));

    //初始Y轴
    motorY.setAcceleration(toStepY(speedval));
    motorY.setSpeed(0);
    motorY.setMaxSpeed(toStepY(speedval / 60));

    // 取当前位置
    float currPos = (dirval==1)?toMMX(motorX.currentPosition()):toMMY(motorY.currentPosition()); 
    float step=0.05;
    float p= currPos;
    float pmax=currPos+maxval; 
    float *parray = new float[10];
    int pindex=0;
    while(p<=pmax){        
        // 移动到目标位置
        if(dirval==1){           
            motorX.runToNewPosition(toStepX(p));
        }else if(dirval==2){
            motorY.runToNewPosition(toStepY(p));
        }

         // 中止查找
        int tValue = analogRead(PIN_MENU_KEY);
        if (tValue < 3000) {
            break;
        }
        
        //获取寻边传感器值
        int psval  = analogRead(PIN_SEEK_BOX);
        //记录值
        parray[pindex]=psval;
        
        
        p=p+step;
        pindex++;
        if(pindex>=10){
            pindex=0;
            //输出
            // grbl_sendf(CLIENT_SERIAL, "VAL:%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,\r\n", parray[0], parray[1], parray[2], parray[3], parray[4], parray[5], parray[6], parray[7], parray[8], parray[9]);
        }
         yield();
    } 
    
    //释放电机
    motors_unstep();
    motors_set_disable(true);

    return 0;
}
