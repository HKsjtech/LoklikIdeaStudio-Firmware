/*
  Grbl_ESP32.ino - Header for system level commands and real-time processes
  Part of Grbl
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

	2018 -	Bart Dring This file was modified for use on the ESP32
					CPU. Do not use this with Grbl for atMega328P

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "src/Grbl.h"
#include "src/Device/SJCutter.h"
#include "src/Device/OTAUpdater.h"

uint8_t otaKey = 0;
extern void make_settings();
void setup() {
    // delay(500);	
    if (!SPIFFS.exists("/")){
      SPIFFS.begin(true);
    }  
    EEPROM.begin(10);
    uint8_t otaFlag = EEPROM.readByte(EEPROM_ADDRESS);
    pinMode(PIN_LED_BLE, OUTPUT);

    if(otaFlag == 0xAA)  
    {
        EEPROM.writeByte(EEPROM_ADDRESS,0xFF);
        delay(10);
        EEPROM.commit();
        otaKey = 1;
        digitalWrite(PIN_LED_BLE, HIGH);
    } else {
      otaKey = 0;
    }

    if(otaKey == 0) {

    #if defined(PIN_POWER)
      ctrlPanelInit();
    #endif
    grbl_init();
    String serial_number = sj_serial_number->get();
    String blueName = "LK_iCraft_";
    blueName = blueName + serial_number.substring(0,4) + "_" + serial_number.substring(18,22);
    BLEGATTServer::getInstance().init(blueName.c_str());
    BLEGATTServer::getInstance().initSemaphore();
    BLEGATTServer::getInstance().otaMode = 0; 
    SJCutterIns.init();

  } else {
    Serial.begin(115200);
    settings_init();  //这是一个系统配置文件
    String serial_number = sj_serial_number->get();
    String otaBlueName = "LK_";
    otaBlueName = otaBlueName + serial_number.substring(0,4) + "_" + serial_number.substring(18,22) + "_" + "V" + FIRMWARE_VERSION + "_OTA";
    BLEGATTServer::getInstance().init(otaBlueName.c_str());
    BLEGATTServer::getInstance().initSemaphore();
    BLEGATTServer::getInstance().otaMode = 1; 

    // 格式化文件分区以保证空间足够;
    FLASH.format();
                    
    SJCutterIns.setCtrlLedAction(BUTTON_BLUE,LEDActions::LED_FAST); 
  }
}

void loop() { 

  if(otaKey == 1) {
    otaUpdataAllTask();
  } else {
    run_once();
  }
}
