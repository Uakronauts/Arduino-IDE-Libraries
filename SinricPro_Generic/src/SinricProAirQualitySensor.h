/*********************************************************************************************************************************
  SinricProAirQualitySensor.h - Sinric Pro Library for boards

  Based on and modified from SinricPro libarary (https://github.com/sinricpro/)
  to support other boards such as SAMD21, SAMD51, Adafruit's nRF52 boards, Teensy, SAM DUE, STM32, etc.

  Built by Khoi Hoang https://github.com/khoih-prog/SinricPro_Generic
  Licensed under MIT license 

  Copyright (c) 2019 Sinric. All rights reserved.
  Licensed under Creative Commons Attribution-Share Alike (CC BY-SA)

  This file is part of the Sinric Pro (https://github.com/sinricpro/)

  Version: 2.8.4
  
  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  2.4.0   K Hoang      21/05/2020 Initial porting to support SAMD21, SAMD51 nRF52 boards, such as AdaFruit Itsy-Bitsy,
                                  Feather, Gemma, Trinket, Hallowing Metro M0/M4, NRF52840 Feather, Itsy-Bitsy, STM32, etc.
  2.5.1   K Hoang      02/08/2020 Add support to STM32F/L/H/G/WB/MP1. Add debug feature, examples. Restructure examples.
                                  Sync with SinricPro v2.5.1: add Speaker SelectInput, Camera. Enable Ethernetx lib support.
  2.6.1   K Hoang      15/08/2020 Sync with SinricPro v2.6.1: add AirQualitySensor, Camera Class.
  2.7.0   K Hoang      06/10/2020 Sync with SinricPro v2.7.0: Added AppKey, AppSecret and DeviceId classes and RTT function.
  2.7.4   K Hoang      12/11/2020 Sync with SinricPro v2.7.4. Add WIO Terminal support and examples
  2.8.0   K Hoang      10/12/2020 Sync with SinricPro v2.8.0. Add examples. Use std::queue instead of QueueList. SSL Option.
  2.8.1   K Hoang      02/06/2021 Add support to RP2040 using Arduino-mbed or arduino-pico core with WiFiNINA or Ethernet
  2.8.2   K Hoang      20/07/2021 Add support to WT32_ETH01 (ESP32 + LAN8720A)
  2.8.3   K Hoang      12/10/2021 Update `platform.ini` and `library.json`
  2.8.4   K Hoang      01/12/2021 Auto detect ESP32 core for LittleFS. Fix bug in examples for WT32_ETH01
 **********************************************************************************************************************************/

#ifndef _SINRIC_PRO_AIRQUALITYSENSOR_H_
#define _SINRIC_PRO_AIRQUALITYSENSOR_H_

#include "SinricProDevice.h"

/**
   @class SinricProAirQualitySensor
   @brief Device to report air quality events
*/
class SinricProAirQualitySensor :  public SinricProDevice 
{
  public:
    SinricProAirQualitySensor(const DeviceId &deviceId);
    
    String getProductType() 
    {
      return SinricProDevice::getProductType() + String("AIR_QUALITY_SENSOR");
    }

    // event
    bool sendAirQualityEvent(int pm1 = 0, int pm2_5 = 0, int pm10 = 0, String cause = "PERIODIC_POLL");
    
  private:
};

SinricProAirQualitySensor::SinricProAirQualitySensor(const DeviceId &deviceId) : SinricProDevice(deviceId) {}

/**
   @brief Sending air quality to SinricPro server

   @param   pm1           1.0 μm particle pollutant in μg/m3
   @param   pm2_5         2.5 μm particle pollutant in μg/m3
   @param   pm10          10 μm particle pollutant in μg/m3
   @param   cause         (optional) `String` reason why event is sent (default = `"PERIODIC_POLL"`)
   @return  the success of sending the event
   @retval  true          event has been sent successfully
   @retval  false         event has not been sent, maybe you sent to much events in a short distance of time
 **/
bool SinricProAirQualitySensor::sendAirQualityEvent(int pm1, int pm2_5, int pm10, String cause) 
{
  DynamicJsonDocument eventMessage = prepareEvent(deviceId, "airQuality", cause.c_str());
  JsonObject event_value = eventMessage["payload"]["value"];

  event_value["pm1"]    = limitValue(pm1, 0, 999);
  event_value["pm2_5"]  = limitValue(pm2_5, 0, 999);
  event_value["pm10"]   = limitValue(pm10, 0, 999);

  return sendEvent(eventMessage);
}

#endif    // _SINRIC_PRO_AIRQUALITYSENSOR_H_
