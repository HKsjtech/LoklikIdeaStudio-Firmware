#ifndef OTAUPDATER_H
#define OTAUPDATER_H

#include <String> 
#include <WString.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h> 
#include <freertos/task.h>  

#include "SPIFFS.h"
#include "Update.h"

#define BUILTINLED 2
#define FORMAT_SPIFFS_IF_FAILED true
#define FORMAT_FFAT_IF_FAILED true

#define USE_SPIFFS        //comment to use FFat
#ifdef USE_SPIFFS
  #define FLASH SPIFFS
  #define FASTMODE false  //SPIFFS write is slow
#else
  #define FLASH FFat
  #define FASTMODE true  //FFat is faster
#endif

extern void otaUpdataAllTask();	// 开机升级任务;

/// 升级工具指令枚举
enum class OTAUpdaterCmd : uint8_t {
    TransferBegin = 0xFA,   //[AA, A1, frameLen(00,00), FA, BinSize(00,00,00,00)，PartCount(00,00), mtuSize(00,00), frameCRC(00,00),0A];接收命令
                            //[AA, A1, frameLen(00,00), FA, OKError(00/E1-EF), curSize(00,00,00,00), frameCRC(00,00),0A];回复
    TransferData  = 0xFB,   //[AA, A1, frameLen(00,00), FB, mtuCount(00,00), DATA(00,....,mtuSize), frameCRC(00,00),0A];接收数据
    TransferCheck = 0xFC,   //[AA, A1, frameLen(00,00), FC, Partsize(00,00), PartCount(00,00), PartCRC(00,00), frameCRC(00,00), 0A];传输检查
                            //[AA, A1, frameLen(00,00), FC, PartSize(00,00), PartCount(00,00), PartCRC(00,00), frameCRC(00,00), 0A];请求part数据
    TransferEnd   = 0xFD,   //[AA, A1, frameLen(00,00), FD, OKError(00/E1-EF), frameCRC(00,00),0A];回复接收完成或错误;
    UpdateBin     = 0xFE,   //[AA, A1, frameLen(00,00), FE, OKError(00/E1-EF), frameCRC(00,00),0A];回复升级完成或错误;从缓存区Bin升级固件 
};


// OTA升级器
class OTAUpdater {
   
private:

  struct updaterData{
    uint8_t upBuff1[16400];
    uint8_t upBuff2[16400];
    bool setBuff;
    bool wirteFile;
    
    uint16_t mtuSize;
    uint16_t upParts;
    uint16_t curPart;
    uint16_t otaStep;
    
    uint16_t partLen1;
    uint16_t partLen2;
    uint32_t binSize;
    uint32_t recSize;
  };

  static  const uint16_t FRAME_SIZE = 512;  //帧最大长度;
  struct frameRxTx{
    uint8_t Buff[FRAME_SIZE];
    static const uint8_t nHead = 0;
    static const uint8_t nType = 1;
    static const uint8_t nLenH = 2;
    static const uint8_t nLenL = 3;
    static const uint8_t nCmd = 4;

    uint16_t Len = 0;
    uint16_t Crc = 0;
    uint8_t Flag = 0;
  }; 

  void handleFormatOTABuffer();
  void handleClearOTABin();
  void handleCacleAvailableSpace();
  void handlePrintAvailableSpace();
  void handleUpdataBin();

  // 私有构造函数，确保外部无法直接创建对象
  OTAUpdater() { }

  // 禁止拷贝构造函数和赋值运算符
  OTAUpdater(const OTAUpdater&);
  OTAUpdater& operator=(const OTAUpdater&);


public:
  // OTAUpdaterStatus updaterStatus;

  updaterData datOTA = {{0}, {0}, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0};

  frameRxTx rxOTA;
  frameRxTx txOTA;

  static OTAUpdater& getInstance();
  // {
  //     static BLEGATTServer instance;
  //     return instance;
  // }

  void writeBinary(fs::FS& fs, const char* path, uint8_t* dat, int len);

  void rebootEspWithReason(String reason);

  void performUpdate(Stream &updateSource, size_t updateSize);

  void updateFromFS(fs::FS &fs);

  void loopHandler();

  //封包发送数据;
  void packetFrameTx(uint8_t *frame,uint8_t *data, uint16_t len);

  /// @brief 读取外设基于BLE传输数据,帧解析;
  void parseFrameRx(frameRxTx* frameR, frameRxTx* frameT);

};

#endif
