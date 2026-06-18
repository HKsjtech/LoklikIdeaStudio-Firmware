#include "../Grbl_Esp32/src/Grbl.h"
#include "../OTAUpdater.h"


/// @brief 静态成员函数，用于获取唯一实例
/// @return 
OTAUpdater& OTAUpdater::getInstance() {
  // 使用静态局部变量确保只初始化一次
  static OTAUpdater instance;
  return instance;
}


/// @param dat 
/// @param len 
void OTAUpdater::writeBinary(fs::FS &fs, const char * path, uint8_t *dat, int len) {

  //Serial.printf("Write binary file %s\r\n", path);

  File file = fs.open(path, FILE_APPEND);

  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  file.write(dat, len);
  file.close();
  // writeFile = false;
  // recSize += len;
}


/// @brief 
/// @param reason 
void OTAUpdater::rebootEspWithReason(String reason) {
  Serial.println(reason);
  delay(1000);
  ESP.restart();
}


/// @param updateSource 
/// @param updateSize 
void OTAUpdater::performUpdate(Stream &updateSource, size_t updateSize) {
  char s1 = 0x0F;
  String result = String(s1);
  if (Update.begin(updateSize)) {
    size_t written = Update.writeStream(updateSource);
    if (written == updateSize) {
      Serial.println("Written : " + String(written) + " successfully");
    }
    else {
      Serial.println("Written only : " + String(written) + "/" + String(updateSize) + ". Retry?");
    }
    result += "Written : " + String(written) + "/" + String(updateSize) + " [" + String((written / updateSize) * 100) + "%] \n";
    if (Update.end()) {
      Serial.println("OTA done!");
      result += "OTA Done: ";
      if (Update.isFinished()) {
        Serial.println("Update successfully completed. ");
        result += "Success!\n";
      }
      else {
        Serial.println("Update not finished? Something went wrong!");
        result += "Failed!\n";
      }

    }
    else {
      Serial.println("Error Occurred. Error #: " + String(Update.getError()));
      result += "Error #: " + String(Update.getError());
    }
  }
  else
  {
    Serial.println("Not enough space to begin OTA");
    result += "Not enough space for OTA";
  }
//   if (BLEGATTServer::getInstance()._deviceConnected) {
//     sendOtaResult(result);
//     delay(5000);
//   }
}


/// @brief 
/// @param fs 
void OTAUpdater::updateFromFS(fs::FS &fs) {
  File updateBin = fs.open("/update.bin");
  if (updateBin) {
    if (updateBin.isDirectory()) {
      Serial.println("Error, update.bin is not a file");
      updateBin.close();
      return;
    }

    size_t updateSize = updateBin.size();

    if (updateSize > 0) {
      Serial.println("Trying to start update");
      performUpdate(updateBin, updateSize);
    }
    else {
      Serial.println("Error, file is empty");
    }

    updateBin.close();

    // when finished remove the binary from spiffs to indicate end of the process
    Serial.println("Removing update file");
    fs.remove("/update.bin");

    // rebootEspWithReason("Rebooting to complete OTA update");
  }
  else {
    Serial.println("Could not load update.bin from spiffs root");
  }
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
uint16_t cal_CRC16(uint8_t *frame, uint16_t usDataLen)  //Recieve frame data to calculate CRC;
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
void OTAUpdater::packetFrameTx(uint8_t *frame,uint8_t *data, uint16_t len) {
  uint8_t length = len + 7;
  uint8_t framePart[4] = {0xAA, 0xA1, uint8_t(length>>8), uint8_t(length)};
  memcpy(&frame[0], &framePart[0], 4);
  memcpy(&frame[4], &data[0], len);
  uint16_t calCRC16 = cal_CRC16(&frame[0], len + 4);
  frame[4 + len + 0] = calCRC16 >> 8;
  frame[4 + len + 1] = calCRC16;
  frame[4 + len + 2] = 0x0A;
}


///
// 完整帧的解析
void OTAUpdater::parseFrameRx(frameRxTx* frameR, frameRxTx* frameT) {

  if(frameR->Flag == 1) {
    // for(int i = 0; i < frameR->Len; i++) {
    //     Serial.printf("%02X ", frameR->Buff[i]);
    // }
    frameR->Len = frameR->Buff[frameR->nLenH] * 256 + frameR->Buff[frameR->nLenL];  // 获取帧长度
    frameR->Crc = frameR->Buff[frameR->Len - 3] * 256 + frameR->Buff[frameR->Len - 2];  // 获取帧的crc
    uint16_t calCRC16 = cal_CRC16(&frameR->Buff[0], frameR->Len - 3); //计算CRC

    if(calCRC16 == frameR->Crc) {
      // Serial.printf("\ncrcOK, ");
      //转换字节为命令枚举
      OTAUpdaterCmd otacmd = static_cast<OTAUpdaterCmd>(frameR->Buff[frameR->nCmd]);

      switch (otacmd) {

        case OTAUpdaterCmd::TransferBegin : {
          for(int i = 0; i < frameR->Len; i++) {
            Serial.printf("%02X ", frameR->Buff[i]);
          }
          // 格式化文件分区
          // FLASH.format();
          if (FLASH.exists("/update.bin. ")) {  // 移除文件
            FLASH.remove("/update.bin. ");
          }
          // part数量 mtuSize大小 bin文件大小
          datOTA.binSize = (frameR->Buff[5] * 256 * 65536) + (frameR->Buff[6] * 65536) + (frameR->Buff[7] * 256) + frameR->Buff[8];
          datOTA.upParts = (frameR->Buff[9] * 256) + frameR->Buff[10];
          datOTA.mtuSize = (frameR->Buff[11] * 256) + frameR->Buff[12];
          datOTA.recSize = 0;
          uint32_t supplySize = (FLASH.totalBytes() - FLASH.usedBytes());
          Serial.printf("supplySize=%uld, binSize=%uld.", supplySize, datOTA.binSize);

          uint16_t dataLen = 6;
          uint8_t txBuff[dataLen] = {0xFA, 0, uint8_t(supplySize>>24), uint8_t(supplySize>>16), uint8_t(supplySize>>8), uint8_t(supplySize)};
          packetFrameTx(&frameT->Buff[0],&txBuff[0], dataLen);
          frameT->Len = dataLen + 7;
          frameT->Flag = 1;
        }
        break;

        case OTAUpdaterCmd::TransferData : {
          // for(int i = 0; i < frameR->Len; i++) {
          // Serial.printf("%02X ", frameR->Buff[i]);
          // }
          //mtuSize单元数据拼接位置
          uint16_t pos = frameR->Buff[5] * 256 + frameR->Buff[6];
          //for循环保存数据到updater缓存
          for (int x = 0; x < frameR->Len - 10; x++) {
            if (datOTA.setBuff) {
              datOTA.upBuff1[(pos * datOTA.mtuSize) + x] = frameR->Buff[x + 7];
            } else {
              datOTA.upBuff2[(pos * datOTA.mtuSize) + x] = frameR->Buff[x + 7];
            }
          }
        }
        break;

        case OTAUpdaterCmd::TransferCheck : {
          
          for(int i = 0; i < frameR->Len; i++) {
            Serial.printf("%02X ", frameR->Buff[i]);
          }

          if (datOTA.setBuff) {
            //写入part长度(大小)
            datOTA.partLen1 = (frameR->Buff[5] * 256) + frameR->Buff[6];
            //计算part数据的CRC16值
            uint16_t ptCRC16 = cal_CRC16(&datOTA.upBuff1[0], datOTA.partLen1);
            //part编号
            datOTA.curPart = frameR->Buff[7] * 256 + frameR->Buff[8];

            Serial.printf("pt%04d,CRC16=0x%04X, ", datOTA.curPart,ptCRC16);

            if(ptCRC16 == (frameR->Buff[9]<<8 | frameR->Buff[10])) {
              Serial.printf("part CRC OK! ");
              
              if(datOTA.curPart < datOTA.upParts - 1) {
                uint16_t dataLen = 5;
                uint8_t txBuff[dataLen] = {0xFC, uint8_t(datOTA.partLen1 >> 8), uint8_t(datOTA.partLen1), uint8_t((datOTA.curPart + 1) >> 8), uint8_t((datOTA.curPart + 1))};
                
                writeBinary(FLASH, "/update.bin", datOTA.upBuff1, datOTA.partLen1);
                datOTA.recSize += datOTA.partLen1;
                
                // 设置接收buff2
                datOTA.setBuff = 0;

                packetFrameTx(&frameT->Buff[0],&txBuff[0], dataLen);
                frameT->Len = dataLen + 7;
                frameT->Flag = 1;   // 请求下一帧
              } else {
                uint16_t dataLen = 2;
                uint8_t txBuff[dataLen] = {0xFD, 0x00};
                
                writeBinary(FLASH, "/update.bin", datOTA.upBuff1, datOTA.partLen1);
                datOTA.recSize += datOTA.partLen1;
                
                Serial.printf("binSize=%ld, recSize=%ld, ", datOTA.binSize, datOTA.recSize);
                if(datOTA.binSize == datOTA.recSize) {
                  txBuff[2] = 0x00;
                  datOTA.otaStep = 9;
                } else {
                  txBuff[2] = 0xE1;
                }
                
                packetFrameTx(&frameT->Buff[0],&txBuff[0], dataLen);
                frameT->Len = dataLen + 7;
                frameT->Flag = 1;// 回复接收成功/失败
              }
            } else {
              Serial.printf("part CRC NG! ");    // 返回原数据，要求重发
              frameT->Len = frameR->Len;
              memcpy(frameT->Buff, frameR->Buff, frameT->Len);
              frameT->Flag = 1;
            }// CRC NG;

          } else {  // buff1
            //写入part长度(大小)
            datOTA.partLen2 = (frameR->Buff[5] * 256) + frameR->Buff[6];
            //计算part数据的CRC16值
            uint16_t ptCRC16 = cal_CRC16(&datOTA.upBuff2[0], datOTA.partLen2);
            //part编号
            datOTA.curPart = frameR->Buff[7] * 256 + frameR->Buff[8];

            Serial.printf("pt%04d,CRC16=0x%04X, ", datOTA.curPart,ptCRC16);

            if(ptCRC16 == (frameR->Buff[9]<<8 | frameR->Buff[10])) {
              Serial.printf("part CRC OK! ");
                  
              if(datOTA.curPart < datOTA.upParts - 1) {
                uint16_t dataLen = 5;
                uint8_t txBuff[dataLen] = {0xFC, uint8_t(datOTA.partLen2 >> 8), uint8_t(datOTA.partLen2), uint8_t((datOTA.curPart + 1) >> 8), uint8_t((datOTA.curPart + 1))};
                
                writeBinary(FLASH, "/update.bin", datOTA.upBuff2, datOTA.partLen2);
                datOTA.recSize += datOTA.partLen2;
                
                // 设置接收buff1
                datOTA.setBuff = 1;
                
                packetFrameTx(&frameT->Buff[0],&txBuff[0], dataLen);
                frameT->Len = dataLen + 7;
                frameT->Flag = 1;   // 请求下一帧
              } else {
                uint16_t dataLen = 2;
                uint8_t txBuff[dataLen] = {0xFD, 0x00};

                writeBinary(FLASH, "/update.bin", datOTA.upBuff2, datOTA.partLen2);
                datOTA.recSize += datOTA.partLen2;

                Serial.printf("binSize=%ld, recSize=%ld, ", datOTA.binSize, datOTA.recSize);
                if(datOTA.binSize == datOTA.recSize) {
                  txBuff[2] = 0x00;
                  datOTA.otaStep = 9;
                } else {
                  txBuff[2] = 0xE1;
                }

                packetFrameTx(&frameT->Buff[0],&txBuff[0], dataLen);
                frameT->Len = dataLen + 7;
                frameT->Flag = 1;   // 回复接收成功/失败
              }
            } else {
              Serial.printf("part CRC NG! ");    // 返回原数据，要求重发
              frameT->Len = frameR->Len;
              memcpy(frameT->Buff, frameR->Buff, frameT->Len);
              frameT->Flag = 1;
            }// CRC NG;
          }// buff2选择
        }
        break;
        default: break;  
      }// switch
    }// 帧的crc检验OK
    frameR->Flag = 0;
  }// if(frameR->Flag == 1) //收到完整帧
}


/// @brief 
void otaUpdataAllTask() 
{
	// 获取 BLEGATTServer 的唯一实例
  BLEGATTServer& u1BLE = BLEGATTServer::getInstance();

  OTAUpdater& u1OTA = OTAUpdater::getInstance();
  unsigned long now = millis();
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

  // 尝试获取信号量
  if (xSemaphoreTake(_bleSemaphoreRead, pdMS_TO_TICKS(portMAX_DELAY)) == pdTRUE) {
        
    // frameRBLE完整帧解析
    if(u1BLE.rxBLE.Flag == 1) {
      u1OTA.rxOTA.Len = u1BLE.rxBLE.Len;
      u1OTA.rxOTA.Flag = u1BLE.rxBLE.Flag;
      memcpy(u1OTA.rxOTA.Buff, u1BLE.rxBLE.Buff, u1OTA.rxOTA.Len);
      u1BLE.rxBLE.Flag = 0;
      
      u1OTA.parseFrameRx(&u1OTA.rxOTA, &u1OTA.txOTA);
      // Serial.printf("Read ok! \n");
      
      if (xSemaphoreTake(_bleSemaphore, pdMS_TO_TICKS(portMAX_DELAY)) == pdTRUE) {
        u1BLE.txBLE.Len = u1OTA.txOTA.Len;
        u1BLE.txBLE.Flag = u1OTA.txOTA.Flag;
        memcpy(u1BLE.txBLE.Buff, u1OTA.txOTA.Buff, u1BLE.txBLE.Len);
        u1OTA.txOTA.Flag = 0;

        if(u1BLE.txBLE.Flag == 1) {
          // for(uint8_t i=0; i<frameT->Len; i++) {
          //     Serial.printf("%02x ", frameT->Buff[i]);
          // }
          u1BLE._pCharacteristicTX->setValue(&u1BLE.txBLE.Buff[0], u1BLE.txBLE.Len);
          u1BLE._pCharacteristicTX->notify();
          u1BLE.txBLE.Flag = 0;
        }

        if(u1OTA.datOTA.otaStep == 9) {
          Serial.printf("Wating for update...");
          u1OTA.updateFromFS(FLASH);
          u1OTA.datOTA.otaStep = 10;
        }

        if(u1OTA.datOTA.otaStep == 10) {
          uint16_t dataLen = 2;
          uint8_t txBuff[dataLen] = {0xFE, 0x00};
          u1OTA.packetFrameTx(&u1BLE.txBLE.Buff[0],&txBuff[0], dataLen);
          u1BLE.txBLE.Len = dataLen + 7;
          u1BLE.txBLE.Flag = 1;   // 回复更新成功/失败
          // for(uint8_t i=0; i<frameT->Len; i++) {
          //     Serial.printf("%02x ", frameT->Buff[i]);
          // }
          u1BLE._pCharacteristicTX->setValue(&u1BLE.txBLE.Buff[0], u1BLE.txBLE.Len);
          u1BLE._pCharacteristicTX->notify();
          delay(1000);
          u1BLE._pCharacteristicTX->setValue(&u1BLE.txBLE.Buff[0], u1BLE.txBLE.Len);
          u1BLE._pCharacteristicTX->notify();
          delay(1000);
          u1BLE._pCharacteristicTX->setValue(&u1BLE.txBLE.Buff[0], u1BLE.txBLE.Len);
          u1BLE._pCharacteristicTX->notify();
          delay(1000);
          u1OTA.datOTA.otaStep = 11;
          u1OTA.rebootEspWithReason("...Rebooting to complete OTA update");
        }
        xSemaphoreGive(_bleSemaphore); // 释放信号量
      }// bleSemaphore信号量
    }// frameRBLE完整帧解析
    xSemaphoreGive(_bleSemaphoreRead); // 释放信号量
  }// bleSemaphoreRead信号量
}
