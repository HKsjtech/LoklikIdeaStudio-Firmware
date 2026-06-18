
#include "../BLEGATTServer.h"
#include <esp32-hal.h> 
#include <HardwareSerial.h>
#include <freertos/semphr.h>
#include <freertos/queue.h> 
#include "../../Grbl.h"  
 


SemaphoreHandle_t _bleSemaphore = NULL;     //保护txBLE发送用;
SemaphoreHandle_t _bleSemaphoreRecv = NULL; //接收回调函数push函数，保护ringBLE用, pop函数保护rxBLE用
SemaphoreHandle_t _bleSemaphoreRead = NULL; //pop函数保护rxBLE用, prase帧解析保护rxBLE用
static QueueHandle_t print_queue=NULL;
static QueueHandle_t print_free=NULL;
static char print_buf[10][128];

int pkg_index = 0;
int rec_index = 0;

/// @brief 将字节数组转为字符串
/// @param data 
/// @param data_len 
/// @param str 
void uint8_to_string(const uint8_t *data, size_t data_len, char *str) {
    for (size_t i = 0; i < data_len; i++) {
        str[i] = (char)data[i];
    }
    str[data_len] = '\0';  // 确保字符串以空字符结尾
}

/// @brief GATT服务回调
class BLEGATTServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        BLEGATTServer::getInstance().setConnectStatus(true);
    }
    void onDisconnect(BLEServer* pServer) {
        BLEGATTServer::getInstance().setConnectStatus(false);
    }
};


///
/// @brief GATT事务回调
class BLEGATTEventCallbacks: public BLECharacteristicCallbacks {
    
    void onWrite(BLECharacteristic *pCharacteristic) {
        if(xSemaphoreTake(_bleSemaphoreRecv, pdMS_TO_TICKS(portMAX_DELAY)) == pdTRUE)
        {
            uint8_t* pData;
            std::string value = pCharacteristic->getValue();
            int len = value.length();
            pData = pCharacteristic->getData();
            if (pData != NULL) {
			    BLEGATTServer& server = BLEGATTServer::getInstance();
                // 压入缓存区
				if(server.otaMode == 0) {
					// 将字节数据转换为字符串
		            char str[len+1];  
		            uint8_to_string(pData,len,str); 
                    // grbl_sendf(CLIENT_SERIAL,"%s",str);
					server.push(str);
				} else if(server.otaMode == 1){
                    // 调用 push 函数，传递 ringBLE 成员变量的地址作为参数
                    int k = server.otaPush(&server.ringBLE, pData, len);
                    // 取出协议帧;
                    int n = server.otaPop(&server.ringBLE, &server.rxBLE);
                    // grbl_sendf(CLIENT_SERIAL,"push:%d,\npop:%d,\n",k, n);
                } else {
                    // 调用 push 函数，传递 ringBLE 成员变量的地址作为参数
                    int k = server.otaPush(&server.ringBLE, pData, len);
                    // 取出协议帧;
                    int n = server.ringPop(&server.ringBLE, &server.rxBLE);
                    // grbl_sendf(CLIENT_SERIAL,"push:%d,\npop:%d,\n",k, n);
                    // 帧解析;为了让实时控制下发放到此处;
                    server.parseFrameRx(&server.rxBLE, &server.txBLE);
                }
            }//(pData != NULL);
            xSemaphoreGive(_bleSemaphoreRecv);
        }//信号量;
	};
    

    ///
	/// @brief GATT事务回调
    void onNotify(BLECharacteristic *pCharacteristic) {
            uint8_t* pData;
            std::string value = pCharacteristic->getValue();
            int len = value.length();
            pData = pCharacteristic->getData();
            // grbl_sendf(CLIENT_SERIAL,"TX:");
            if (pData != NULL) {
                // for (int i = 0; i < len; i++) {
                //     grbl_sendf(CLIENT_SERIAL,"%c", pData[i]);
                // }
            }
    }
}; 

/// @brief 静态成员函数，用于获取唯一实例
/// @return 
BLEGATTServer& BLEGATTServer::getInstance() {
    // 使用静态局部变量确保只初始化一次
    static BLEGATTServer instance;
    return instance;
}

// 启动广播的辅助函数
void startAdvertising() {
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
}

///
/// @brief 初始化BLE
void BLEGATTServer::init(String blename) {

    // 创建BLE服务并广播ID
    BLEDevice::init(blename.c_str());
    pBleServer = BLEDevice::createServer();
    pBleServer->setCallbacks(new BLEGATTServerCallbacks());
    pService = pBleServer->createService(SERVICE_UUID);
    // 配置服务支持写入特性
    _pCharacteristicRX = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    _pCharacteristicRX->setCallbacks(new BLEGATTEventCallbacks());
    // 配置服务支持读取特性
    _pCharacteristicTX = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    _pCharacteristicTX->setCallbacks(new BLEGATTEventCallbacks());
    _pCharacteristicTX->addDescriptor(new BLE2902());
    _pCharacteristicTX->setNotifyProperty(true);
    
    pService->start();

    startAdvertising();
    Serial.println("Characteristic defined! Now you can read it in your phone!");
	
    // 变更初始化状态
    _initStatus=true;
}

void BLEGATTServer::initSemaphore(void) {
    // 创建二值信号量 _bleSemaphore
    _bleSemaphore = xSemaphoreCreateBinary();
    if (_bleSemaphore != NULL) {
        // 如果信号量创建成功，给予信号量一个“给予”操作，以使其初始状态为可用
        xSemaphoreGive(_bleSemaphore);
    } else {
        ESP_LOGE("INIT", "Failed to create _bleSemaphore");
    }

    // 创建二值信号量 _bleSemaphoreRecv
    _bleSemaphoreRecv = xSemaphoreCreateBinary();
    if (_bleSemaphoreRecv != NULL) {
        // 如果信号量创建成功，给予信号量一个“给予”操作，以使其初始状态为可用
        xSemaphoreGive(_bleSemaphoreRecv);
    } else {
        ESP_LOGE("INIT", "Failed to create _bleSemaphoreRecv");
    }

    // 创建二值信号量 _bleSemaphoreRead
    _bleSemaphoreRead = xSemaphoreCreateBinary();
    if (_bleSemaphoreRead != NULL) {
        // 如果信号量创建成功，给予信号量一个“给予”操作，以使其初始状态为可用
        xSemaphoreGive(_bleSemaphoreRead);
    } else {
        ESP_LOGE("INIT", "Failed to create _bleSemaphoreRead");
    }
}

/// @brief 设置蓝牙服务状态
/// @param state 
void BLEGATTServer::setBleFindStatus(bool state){
    if(state && !_initStatus)
    {
        _initStatus=true;
        startAdvertising();
    }
    else if(!state && _initStatus)
    {
        _initStatus = false;
        BLEDevice::getAdvertising()->stop();
    }
}

/// @brief 设置外设连接状态
/// @param state 
void BLEGATTServer::setConnectStatus(bool state){
    _deviceConnected = state;
}

/// @brief 设置外设连接状态 
bool BLEGATTServer::hasClient(){
    return _deviceConnected;
}

/// @brief 通过BLE向外设发送数据
/// @param result 
void BLEGATTServer::send(const char* result) {
    if(!_initStatus)return; 

    if (xSemaphoreTake(_bleSemaphore, pdMS_TO_TICKS(portMAX_DELAY)) == pdTRUE) {        
        String subStr = result;
        // char* subStr = (char*)malloc(strlen(result) + 1);
        // strncpy(subStr, result, strlen(result));
        int numSegments = strlen(result) / 20 + 1;
        // char temp[20] = {0};
        for (int i = 0; i < numSegments; i++)
        {
            // strncpy(temp, subStr + i * 20, 20);
            String segment = subStr.substring(i * 20, (i + 1) * 20); // 截取20个字符的段

            // String segment = temp;
            _pCharacteristicTX->setValue(segment.c_str());
            _pCharacteristicTX->notify();
            delay(3); // 延迟一段时间，以确保传输完成
        }
        xSemaphoreGive(_bleSemaphore);
    }
    // free((char*)subStr);
    // delay(200);
}


bool BLEGATTServer::push(const char* data) {
    int data_size = strlen(data);
    if ((data_size + _RXbufferSize) <= RXBUFFERSIZE) {
        int current = _RXbufferpos + _RXbufferSize;
        if (current > RXBUFFERSIZE) {
            current = current - RXBUFFERSIZE;
        }
        for (int i = 0; i < data_size; i++) {
            if (current > (RXBUFFERSIZE - 1)) {
                current = 0;
            }
            _RXbuffer[current] = data[i];
            current++;
        }
        _RXbufferSize += strlen(data);
        return true;
    }
    return false;
}

int BLEGATTServer::read(void) {
    if(xSemaphoreTake(_bleSemaphoreRecv, pdMS_TO_TICKS(portMAX_DELAY)) == pdTRUE)
    {
        if (_RXbufferSize > 0) {
            int v = _RXbuffer[_RXbufferpos];
            // grbl_sendf(CLIENT_SERIAL, "%c",v);
            _RXbufferpos++;
            if (_RXbufferpos > (RXBUFFERSIZE - 1)) {
                _RXbufferpos = 0;
            }
            _RXbufferSize--;
            xSemaphoreGive(_bleSemaphoreRecv);
            return v;
        } else {
            xSemaphoreGive(_bleSemaphoreRecv);
            return -1;
        }
    }
}

/// @brief 数据存入缓冲环
/// @param ring 
/// @param data 
/// @param len 
/// @return 
bool BLEGATTServer::otaPush(ringRec* ring, uint8_t* data, uint16_t len) {

    int16_t rxLen = len;
    // 检查缓冲区是否有足够空间
    if (ring->Count + rxLen > RING_SIZE) {
        Serial.println("otaPush overflow.\n");
        return false; // 错误：缓冲区溢出，返回 false
    } else {
        // 检查写入的数据是否会跨越缓冲区边界
        if (ring->Tail + rxLen <= RING_SIZE) {
            // 不会跨越缓冲区边界，可以直接写入
            memcpy(&ring->Buff[ring->Tail], &data[0], rxLen);
            ring->Tail = (ring->Tail + rxLen) % RING_SIZE;
        } else {
            // 会跨越缓冲区边界，需要分段写入
            uint16_t len1 = RING_SIZE - ring->Tail; // 缓冲区末尾的剩余空间
            uint16_t len2 = rxLen - len1;           // 需要从缓冲区开头写入的数据长度
            memcpy(&ring->Buff[ring->Tail], &data[0], len1);
            memcpy(&ring->Buff[0], &data[len1], len2);
            ring->Tail = len2; // 更新 Tail 为第二段数据结束的位置
        }
        ring->Count += rxLen;  // 更新缓冲区中的数据计数
        return true; // 成功
    }
}


/// @brief 从缓冲环取出完整帧
/// @param ring 
/// @param frameR 
/// @return 
int BLEGATTServer::otaPop(ringRec* ring, frameRxTx* frameR) {
    // 接收缓冲环的信号量
    // if (xSemaphoreTake(_bleSemaphoreRecv, pdMS_TO_TICKS(portMAX_DELAY)) == pdTRUE) {  
        // 确保有足够的数据来检查帧头和长度字段
        while (ring->Count > 6) { 
            // 0xAAA1协议头
            if (ring->Buff[ring->Head] == 0xAA) {
            // if ((ring->Buff[ring->Head] == 0xAA) && (ring->Buff[(ring->Head + 1) % RING_SIZE] == 0xA1)) {
                uint16_t lenByte1 = (ring->Head + 2) % RING_SIZE;
                uint16_t lenByte2 = (ring->Head + 3) % RING_SIZE;
                // 尝试获取帧的信号量
				if (xSemaphoreTake(_bleSemaphoreRead, pdMS_TO_TICKS(portMAX_DELAY)) == pdTRUE) {
                    frameR->Len = ring->Buff[lenByte1] * 256 + ring->Buff[lenByte2];
                    // 检查是否有足够的数据读取整个帧
                    if (frameR->Len <= ring->Count) {  
                        uint16_t frameREnd = (ring->Head + frameR->Len - 1) % RING_SIZE;
                        // 检查帧结束标志
                        if (ring->Buff[frameREnd] == 0x0A) {  
                            for (uint16_t k = 0; k < frameR->Len; k++) {
                                frameR->Buff[k] = ring->Buff[(ring->Head + k) % RING_SIZE];
                            }
                            // 更新缓冲区头指针和计数器
                            ring->Head = (ring->Head + frameR->Len) % RING_SIZE;
                            ring->Count -= frameR->Len;
                            frameR->Flag = 1;

                            xSemaphoreGive(_bleSemaphoreRead);
                            // xSemaphoreGive(_bleSemaphoreRecv);
                            return true;  // 成功找到并处理帧
                        }  else {
                            // 找到协议头但帧结束标志不正确，自加，再跳出循环。
                            ring->Head = (ring->Head + 1) % RING_SIZE;
                            ring->Count--;
                        }
                    } else {
                        // 数据不完整，等待更多数据
                        xSemaphoreGive(_bleSemaphoreRead);
                        break;
                    }
                    xSemaphoreGive(_bleSemaphoreRead);
                } else {
                    // 如果无法获取帧的信号量，则释放接收缓冲环的信号量
                    // xSemaphoreGive(_bleSemaphoreRecv);
                    return false;
                }
            } else {
                // 没有找到协议头，移动到下一个字节
                ring->Head = (ring->Head + 1) % RING_SIZE;
                ring->Count--;
            }
        }// while
        // xSemaphoreGive(_bleSemaphoreRecv);
        return false; // 返回 false
    // }
    return false; // 无法获取信号量，返回 false
}


/// @brief 返回接收的数据
/// @param frameR 
/// @param frameT 
/// @return 
int BLEGATTServer::otaParse(frameRxTx* frameR, frameRxTx* frameT) {
    if (xSemaphoreTake(_bleSemaphoreRead, pdMS_TO_TICKS(portMAX_DELAY)) == pdTRUE) {
        frameT->Len = frameR->Len;
        memcpy(frameT->Buff, frameR->Buff, frameT->Len);

        _pCharacteristicTX->setValue(frameT->Buff, frameT->Len);
        _pCharacteristicTX->notify();

        // char strdata[256];
        // uint8_to_string(frameT->Buff, frameT->Len, strdata);
        // send(strdata);

        xSemaphoreGive(_bleSemaphoreRead); // 释放信号量
        return false;
    }// 获取信号量
    return false;
}


///
///Table Of CRC Values for high-order byte
//================================================================//========================================//
static const uint8_t auchCRCHi[256] = {
   0, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
};


///
//Table of CRC values for low-order byte;
//================================================================//========================================//
static const uint8_t auchCRCLo[256] = {
   0, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 
0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 
0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 
0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 
0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 
0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 
0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 
0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 
0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 
0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 
0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 
0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 
0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 
0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40, 
};


///
///CRC Calculate;
//================================================================//========================================//
uint16_t BLEGATTServer::calculateCRC16(uint8_t *frame, uint16_t usDataLen)  //Recieve frame data to calculate CRC;
{											
	uint16_t uCRC_Value = 0;
	uint8_t uchCRCHi  = 0xFF ;  //high byte of CRC init;
	uint8_t uchCRCLo  = 0xFF ;  //low byte of CRC init;
	uint8_t uIndex ;            //index into CRC lookup table;
	while(usDataLen--)          //Pass through message buffer;
	{												
        uIndex   = uchCRCHi ^ *frame++ ;          //calculate the CRC;
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
		uchCRCLo = auchCRCLo[uIndex] ;
	}
    uCRC_Value = (((uint16_t) uchCRCLo << 8) | uchCRCHi);    //内存及协议低字节在前
	return uCRC_Value;  //CRC high and Low bit change;
}


/// @param 
/// @param frame data len 
void BLEGATTServer::packetFrameTx(uint8_t *frame,uint8_t *data, uint16_t len) {
  uint8_t length = len + 7;
  uint8_t framePart[4] = {0xAA, 0xA2, uint8_t(length>>8), uint8_t(length)};
  memcpy(&frame[0], &framePart[0], 4);
  memcpy(&frame[4], &data[0], len);
  uint16_t calCRC16 = calculateCRC16(&frame[0], len + 4);
  frame[4 + len + 0] = calCRC16 >> 8;
  frame[4 + len + 1] = calCRC16;
  frame[4 + len + 2] = 0x0A;
}


/// @brief 从缓冲环取出完整帧
/// @param ring 
/// @param frameR 
/// @return 
int BLEGATTServer::ringPop(ringRec* ring, frameRxTx* frameR) {
    // 确保有足够的数据来检查帧头和长度字段
    while (ring->Count > 6) { 
        // 0xAAA1协议头
        if (ring->Buff[ring->Head] == 0xAA) {
        // if ((ring->Buff[ring->Head] == 0xAA) && (ring->Buff[(ring->Head + 1) % RING_SIZE] == 0xA1)) {
            uint16_t lenByte1 = (ring->Head + 2) % RING_SIZE;
            uint16_t lenByte2 = (ring->Head + 3) % RING_SIZE;
            uint16_t frameLen = (ring->Buff[lenByte1] << 8) + ring->Buff[lenByte2];
            
            // 检查是否有足够的数据读取整个帧
            if (frameLen <= ring->Count) {  
                uint16_t frameREnd = (ring->Head + frameLen - 1) % RING_SIZE;
                // 检查帧结束标志
                if (ring->Buff[frameREnd] == 0x0A) {
                    // 获取信号量，一次完成所有帧操作
                    // if (xSemaphoreTake(_bleSemaphoreRead, pdMS_TO_TICKS(portMAX_DELAY)) == pdTRUE) {
                        // 复制帧数据
                        for (uint16_t k = 0; k < frameLen; k++) {
                            frameR->Buff[k] = ring->Buff[(ring->Head + k) % RING_SIZE];
                        }
                        // 更新缓冲区头指针和计数器
                        ring->Head = (ring->Head + frameLen) % RING_SIZE;
                        ring->Count -= frameLen;
                        frameR->Len = frameLen;
                        frameR->Flag = 1;
                        // 释放信号量
                        // xSemaphoreGive(_bleSemaphoreRead);
                        return true;  // 成功找到并处理帧
                    // }// 信号量;
                }// 0x0A结尾;
            } // 帧数据长度符合;
        } // 协议头0xAA;
        // 如果协议头不正确，或者帧结束不正确，移动到下一个字节
        ring->Head = (ring->Head + 1) % RING_SIZE;
        ring->Count--;
    }
    return false;  // 没有找到符合条件的帧
}

///
// 完整帧的解析
bool BLEGATTServer::parseFrameRx(frameRxTx* frameR, frameRxTx* frameT) {

  BLEGATTServer& u1BLE = BLEGATTServer::getInstance();

  if(frameR->Flag == 1) {
    
    frameR->Len = frameR->Buff[frameR->nLenH] * 256 + frameR->Buff[frameR->nLenL];      // 获取帧长度
    frameR->Crc = frameR->Buff[frameR->Len - 3] * 256 + frameR->Buff[frameR->Len - 2];  // 获取帧的crc
    uint16_t calCRC16 = calculateCRC16(&frameR->Buff[0], frameR->Len - 3);              //计算CRC
    
    // grbl_sendf(CLIENT_SERIAL,"parse:");
    
    if(calCRC16 == frameR->Crc) {
      // Serial.printf("\ncrcOK, ");
      blockDataCmd blkcmd = static_cast<blockDataCmd>(frameR->Buff[frameR->nCmd]);
      // 转换字节为命令枚举;
      switch (blkcmd) {

        case blockDataCmd::TransferBegin : {
        }   //
        break;

        case blockDataCmd::TransferData : {
            // FB;
            uint16_t rxLen = frameR->Len - 10;
            // 检查缓冲区是否有足够空间
            if (ringGD0.Count + rxLen > GCODE_SIZE) {
                // 缓冲区溢出处理，这里假设丢弃新数据
                grbl_sendf(CLIENT_SERIAL,"ringGD0 overflow.\n");
                return false;           //缓冲区溢出，返回 false
            } else {
                // 检查写入的数据是否会跨越缓冲区边界
                if (ringGD0.Tail + rxLen <= GCODE_SIZE) {
                    // 不会跨越缓冲区边界，可以直接写入
                    memcpy(&ringGD0.Buff[ringGD0.Tail], &frameR->Buff[7], rxLen);
                    ringGD0.Tail = (ringGD0.Tail + rxLen) % GCODE_SIZE;
                } else {
                    // 会跨越缓冲区边界，需要分段写入
                    uint16_t len1 = GCODE_SIZE - ringGD0.Tail; // 缓冲区末尾的剩余空间
                    uint16_t len2 = rxLen - len1;           // 需要从缓冲区开头写入的数据长度
                    memcpy(&ringGD0.Buff[ringGD0.Tail], &frameR->Buff[7], len1);
                    memcpy(&ringGD0.Buff[0], &frameR->Buff[len1 + 7], len2);
                    ringGD0.Tail = len2; // 更新 Tail 为第二段数据结束的位置
                }
                ringGD0.Count += rxLen; // 更新缓冲区中的数据计数
                // grbl_sendf(CLIENT_SERIAL,"frameR>ringGD0.\n");
                frameR->Flag = 0;       // 正常解析;
                ringGD0.Flag = 1;       // 正常存放;
                return true;            // 正确退出;
            }
        }
        break;

        case blockDataCmd::TransferCheck : {
            //执行缓冲
            uint16_t rxLen = frameR->Len - 8;
            uint8_t* pData;
            char str[rxLen+1];

            pData = &frameR->Buff[5];
            uint8_to_string(pData, rxLen, str);
            
            bool pushS = 0;
            // if(read() == -1)//执行缓冲无数据;
                pushS = u1BLE.push(str);
            if(pushS == 1) {
                // grbl_sendf(CLIENT_SERIAL,"frameR>ringGD0:%s\n",str);
                frameR->Flag = 0;   // 正常;
                return true;        // 正确退出;
            } else {
                // grbl_sendf(CLIENT_SERIAL,"frameR>ringGD0:X\n",str);
                return false;
            }
        }
        break;

        default: break;  
      }// switch
    }// 帧的crc检验OK
    // frameR->Flag = 0;
  }// if(frameR->Flag == 1) //收到完整帧，且未完成正常解析;
}


/// @brief 
void blockDataProcess(void *pvParameters) {
    // for(;;) {
        // 获取 BLEGATTServer 的唯一实例
        BLEGATTServer& u1BLE = BLEGATTServer::getInstance();

        if(xSemaphoreTake(_bleSemaphoreRecv, pdMS_TO_TICKS(portMAX_DELAY)) == pdTRUE)
        {
            if((u1BLE.ringGD0.Flag == 1) && (u1BLE.ringGD0.Count < u1BLE.ringGD0.Leng - 1024)){
                // 剩余空间>1024;
                if (xSemaphoreTake(_bleSemaphore, pdMS_TO_TICKS(portMAX_DELAY)) == pdTRUE) {
                    uint8_t txBuff[] = "REQ:0\r\n";
                    u1BLE._pCharacteristicTX->setValue(&txBuff[0], 7);
                    u1BLE._pCharacteristicTX->notify();
                    u1BLE.ringGD0.Flag = 0;
                    u1BLE.ringGD0.Pack ++;
                    delay(3);
                    xSemaphoreGive(_bleSemaphore); // 释放信号量
                }
            }

            if(u1BLE.ringGD0.Count > u1BLE.ringGD0.Leng - 1024) {
                u1BLE.ringGD0.Start = 1;
            }// 首次使用数据存入Leng-1024;

            if((u1BLE.ringGD0.Start == 1) && (u1BLE.ringGD0.Count > 0)) {

                uint16_t rxLen = 0;
                char str[128];
                memset(&str[0], '\0', 128);
                
                for(uint16_t i=0; i<u1BLE.ringGD0.Count; i++){
                    str[i] = (char)u1BLE.ringGD0.Buff[(i+u1BLE.ringGD0.Head) % u1BLE.ringGD0.Leng];
                    // grbl_sendf(CLIENT_SERIAL,"%c", str[i]);
                    if(str[i] == '\n') {
                        rxLen = i + 1;  //找到换行符; 
                        break;
                    }
                }

                // if(xSemaphoreTake(_bleSemaphoreRecv, pdMS_TO_TICKS(portMAX_DELAY)) == pdTRUE)
                // {
                    bool Sig = u1BLE.push(str);
                    if(Sig == 1) {
                        u1BLE.ringGD0.Count -= rxLen;
                        u1BLE.ringGD0.Head = (u1BLE.ringGD0.Head + rxLen) % u1BLE.ringGD0.Leng;
                        // grbl_sendf(CLIENT_SERIAL,"%04d:%s",u1BLE.ringGD0.Head,str);
                        delay_ms(1);
                    }
                //     xSemaphoreGive(_bleSemaphoreRecv);
                // }
            }
            xSemaphoreGive(_bleSemaphoreRecv);
        }
    // vTaskDelay(15);
    // }// for;
}
