#pragma once
 
#include <String> 
#include <WString.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h> 
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h> 
#include <freertos/task.h>  

#include "SPIFFS.h"
#include "Update.h"


    #define BLE_OTA_NAME            "LK_GB01_XABE_V02_OTA"
    #define SERVICE_UUID            "00010203-0405-0607-0809-0a0b0c0dffe0"
    #define CHARACTERISTIC_UUID_RX  "00010203-0405-0607-0809-0a0b0c0dffe2"
    #define CHARACTERISTIC_UUID_TX  "00010203-0405-0607-0809-0a0b0c0dffe1"

#define GATT_PACKET_QUEUELEN   50
// ble拼包缓存最大尺寸        
const uint16_t GATT_RX_BUFFER_MAX = 512;


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

typedef struct {
        size_t len;
        uint8_t data[];
} gatt_packet_t;
 
extern void blockDataProcess(void *pvParameters);	// 数据块发任务;
// extern void blockDataProcessTask(); //数据块发os Task;

/// 升级工具指令枚举
enum class blockDataCmd : uint8_t {
    TransferBegin = 0xFA,   //[AA, A2, frameLen(00,00), FA, BinSize(00,00,00,00)，PartCount(00,00), mtuSize(00,00), frameCRC(00,00),0A];接收命令
                            //[AA, A2, frameLen(00,00), FA, OKError(00/E1-EF), curSize(00,00,00,00), frameCRC(00,00),0A];回复
    TransferData  = 0xFB,   //[AA, A2, frameLen(00,00), FB, mtuCount(00,00), DATA(00,....,mtuSize), frameCRC(00,00),0A];接收数据
    TransferCheck = 0xFC,   //[AA, A2, frameLen(00,00), FC, Partsize(00,00), PartCount(00,00), PartCRC(00,00), frameCRC(00,00), 0A];传输检查
                            //[AA, A2, frameLen(00,00), FC, PartSize(00,00), PartCount(00,00), PartCRC(00,00), frameCRC(00,00), 0A];请求part数据
    TransferEnd   = 0xFD,   //[AA, A2, frameLen(00,00), FD, OKError(00/E1-EF), frameCRC(00,00),0A];回复接收完成或错误;
    UpdateBin     = 0xFE,   //[AA, A2, frameLen(00,00), FE, OKError(00/E1-EF), frameCRC(00,00),0A];回复升级完成或错误;从缓存区Bin升级固件 
};


extern SemaphoreHandle_t _bleSemaphore;     //保护txBLE发送用;
extern SemaphoreHandle_t _bleSemaphoreRecv; //接收回调函数push函数，保护ringBLE用, pop函数保护rxBLE用
extern SemaphoreHandle_t _bleSemaphoreRead; //pop函数保护rxBLE用, prase帧解析保护rxBLE用


///
/// 蓝牙服务类
class BLEGATTServer{
         
    private:
        // 初始化状态
        bool _initStatus = false;

        static const uint16_t RING_SIZE = 8192;  //缓冲环最大长度;
        struct ringRec{
            uint8_t Buff[RING_SIZE];
            uint16_t Head;
            uint16_t Tail;
            uint16_t Count;
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

        static const int RXBUFFERSIZE = 256;

        uint8_t  _RXbuffer[RXBUFFERSIZE];
        uint16_t _RXbufferSize;
        uint16_t _RXbufferpos;
        
        // BLECharacteristic* _pCharacteristicTX;
        // BLECharacteristic* _pCharacteristicRX;
        
        // 接收数据处理队列 
        QueueHandle_t _xQueue;
    
        // 私有构造函数，确保外部无法直接创建对象
        BLEGATTServer() { }

        // 禁止拷贝构造函数和赋值运算符
        BLEGATTServer(const BLEGATTServer&);
        BLEGATTServer& operator=(const BLEGATTServer&);

        static  const uint16_t GCODE_SIZE = 4096;  //gcode buff最大长度;
        struct ringGCODE{
            // EXT_RAM_ATTR uint8_t Buff[GCODE_SIZE];
            uint8_t Buff[GCODE_SIZE];
            uint16_t Head;
            uint16_t Tail;
            uint16_t Count;
            uint16_t Leng = GCODE_SIZE;
            uint16_t Pack;
            uint8_t Flag;
            uint8_t Start;
        };
 
    public:
		// 连接状态;
        bool _deviceConnected = false;
		
        BLECharacteristic* _pCharacteristicTX;
        BLECharacteristic* _pCharacteristicRX;
        BLEService *pService;
        BLEServer *pBleServer;
        ringRec ringBLE = { {0}, 0, 0, 0 };

        frameRxTx rxBLE;
        frameRxTx txBLE;
        uint8_t otaMode = 0;

        ringGCODE ringGD0;
        // ringGCODE ringGD1;
        

        // //Table Of CRC Values for high-order byte
        // static const uint8_t auchCRCHi[256];
        // //Table of CRC values for low-order byte;
        // static const uint8_t auchCRCLo[256];
        // CRC Calculate;//Recieve frame data to calculate CRC;
        // uint16_t cal_CRC16(uint8_t *frame, uint16_t usDataLen);

        // 静态成员函数，用于获取唯一实例
        static BLEGATTServer& getInstance();
        // {
        //     static BLEGATTServer instance;
        //     return instance;
        // }
         
        /// @brief 初始化BLE
        void init(String blename);

        /// @brief 设置外设连接状态
        /// @param state 
        void setConnectStatus(bool state);

        /// @brief 向队列中压入BLE接收数据
        /// @param state 
        esp_err_t pushReceivePacketToQueue(uint8_t *data,size_t len);


        /// @brief 通过BLE向外设发送数据
        /// @param result 
        void send(const char* result);

        /// @brief 读取外设基于BLE传输数据
        /// @return 
        int read();

        /// @brief 返回是否有外设连接
        /// @return 
        bool hasClient();
        
        
        /// @brief 压入数据到缓存区
        /// @param data 
        /// @return 
        bool push(const char* data);

        /// @brief 设置蓝牙服务状态
        /// @param state 
        void setBleFindStatus(bool state);
        
        char* readWithQueue(void);
        void freeQueue(char* buf);
		
		/// @brief 
        /// @param data 
        /// @param data_len 
        /// @param str 
        // void uint8_to_string(const uint8_t *data, size_t data_len, char *str);

        
        /// @brief 压入数据到缓存区
        /// @param ring
        /// @return 
        bool otaPush(ringRec* ring, uint8_t* data, uint16_t len);

        /// @brief 从缓冲环调出一帧数据
        /// @param ring 缓冲环结构体指针
        /// @param frame 帧数据结构体指针
        /// @return 是否成功找到并处理帧
        int otaPop(ringRec* ring, frameRxTx* frameR);   
        
        /// @brief 读取外设基于BLE传输数据,帧解析;
        /// @return 
        int otaParse(frameRxTx* frameR, frameRxTx* frameT);

        /// @param 
        /// @param frame data len
        uint16_t calculateCRC16(uint8_t *frame, uint16_t usDataLen);  //Recieve frame data to calculate CRC;
        
        /// @param 
        /// @param frame data len 
        void packetFrameTx(uint8_t *frame,uint8_t *data, uint16_t len);

        /// @brief BLE传输数据,帧解析;
        /// @return 
        int ringPop(ringRec* ring, frameRxTx* frameR);

        /// @brief BLE传输数据,帧解析;
        /// @return 
        bool parseFrameRx(frameRxTx* frameR, frameRxTx* frameT);

        void initSemaphore(void);
};
