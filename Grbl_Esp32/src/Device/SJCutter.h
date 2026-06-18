// Copyright (c) 2022 -	Neil 

#pragma once
#ifndef SJCUTTER_H
#define SJCUTTER_H


#include "../Grbl.h" 
#include <cstdint> 


#define IO_SECURITY_PKG_SIZE            22//加密响应包大小
#define IO_SECURITY_SUMVAL              113//安全和值
#define IO_SECURITY_REQUEST_MARK        62//请求包标识
#define IO_SECURITY_RESPONSE_MARK       35//响应包标识
#define IO_SECURITY_ALERT_LOST_PKG_MARK 33//丢包警告标识
#define IO_SECURITY_LINE_PKG_MARK 35// 行标识

#define BUTTON_POWER		0
#define BUTTON_PAUSE		1
#define BUTTON_PAPER		2
#define BUTTON_START		3
#define BUTTON_BLUE     4
#define BUTTON_USB      5

#define EEPROM_ADDRESS 1

#define HEARTBEAT_TIMEOUT_MS 3000UL  // 3秒内未收到?则认为连接断开

/// 指示灯状态
typedef enum{
  LED_OFF = 0,
  LED_ON = 1,
  LED_SLOW = 2,
  LED_FAST = 3
} LEDActions;

typedef enum{
  PAPER_BTN_MODE_DEF = 0,
  PAPER_BTN_MODE_ADJ = 1,
} PAPER_BTN_MODE;
/// 指令执行状态
typedef enum{
  RESULT_FAIL = 0,
  RESULT_OK = 1,
  RESULT_IGNORE = 2,
  RESULT_ABANDON = 3,
  RESULT_RESPONSE_OK = 4,
  RESULT_RESPONSE_FAIL = 5,S
} CMDResult;

/// 切割机实现
class SJCutter {
public:
    // 行数据最大字节数
    static const int maxLine = 255;
    // 测试模式
    bool testMode = false;
    // 开关状态
    u_int powerStatus = false;
    // 机器工作状态
    bool readyStatus = false;
    // X轴归为状态
    bool xHomeStatus = true;
    // Z轴归为状态
    bool zHomeStatus = true;
    // 物料加载状态
    bool mateLoadStatus = false;
    // 任务运行状态
    bool taskRunStatus = false;    
    // 任务暂停状态
    bool taskPausedStatus = false;
    // 智能膜材退料标记
    bool smartMaterial = false;
    // SN写入标记
    bool snUpdateTag = false;
    // 安全传输码
    uint8_t  taskSecurityCode =1;
    // 最后接收行号
    u_int   taskRecvLine =0;    
    // 任务执行速度
    u_int   taskSpeed =5000;
    u_int   moveSpeed =5000;
    // 最后发生JOG状态时间
    unsigned long   lastJogTime =0;

    // 开启时长
    unsigned long powerOnTime = millis();
    // LED集合
    u_int ledSet[6][2]={
        {PIN_LED_POWER, LEDActions::LED_OFF},
        {PIN_LED_PAUSE, LEDActions::LED_OFF},
        {PIN_LED_PAPER, LEDActions::LED_OFF},
        {PIN_LED_START, LEDActions::LED_OFF},
        {PIN_LED_BLE, LEDActions::LED_OFF},
        {PIN_LED_USB, LEDActions::LED_OFF},
        
    };
    
    // 按钮集合
    u_int buttonSet[4][2]={
        {PIN_LED_POWER, LOW},
        {PIN_LED_PAUSE, LOW},
        {PIN_LED_PAPER, LOW},
        {PIN_LED_START, LOW},
    }; 
 

protected:
    const char* _name;             

public:
    SJCutter();
    ~SJCutter(); 
    // 初始化
    void init();
    // 事务循环
    void loop();
    // 指令消息处理
    CMDResult handleMessage(uint8_t client,char* line); 
    // 强制等待状态模式
    bool waitForStatus();
    // 安全处理
    CMDResult securityProcess(uint8_t client,char* line);
    
    // 修改控制端发送的指令
    void modifyCommendLine(char* line);
    // 设备按钮状态
    void setButtonST(u_int key,bool val);
    void resetButtonST();
    // 获得取按钮状态
    bool getButtonST(u_int key);
    // 设置LED
    void setCtrlLedAction(int key,int action);
    // 变更物料加载状态
    void exchangeMateLoadStatus(uint8_t client);
    // 变更任务暂停状态
    void exchangeTaskPausedStatus(uint8_t client);
     // 读取序列数据
    void readSerialNumber(uint8_t client);
    // 读取固件版本
    void readFirmwareVersion(uint8_t client);
    // 读取步长数
    void readStepsPerMM(uint8_t client);
    // 写入配置
    bool writeStepsPerMM(uint8_t client,char* line);
    // 写入Z配置
    bool writeZOffset(uint8_t client,char* line);
    // 读取状态数据
    void readStatus(char* status_info,uint8_t status_tag);    
    // 数据解码
    bool decodeData(uint8_t client,char* line);
    // 调整Z轴值 
    void adjustZValue(bool isG0, char* line);    
    // 重置状态 
    void reboot();
    // 检查led灯状态
    void cheakLedConnectStatus(uint8_t client);
    void cheakLedDisConnectStatus();
    void TaskClear(uint8_t client);
    void writeRangeCompen(char* line);
    void calcuRangeCompen();
    float returnCompen(float x);
};

// 
void ctrlPanelInit();
void ctrlPanelLoop();

extern SJCutter SJCutterIns;

#endif