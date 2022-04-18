/****************************************************************************************************************************
  nRF52.ino
  For Adafruit nRF52 boards, running W5x00 or ENC28J60 Ethernet shield

  Based on and modified from SinricPro libarary (https://github.com/sinricpro/)
  to support other boards such as  SAMD21, SAMD51, Adafruit's nRF52 boards, etc.

  Built by Khoi Hoang https://github.com/khoih-prog/SinricPro_Generic
  Licensed under MIT license

  Copyright (c) 2019 Sinric. All rights reserved.
  Licensed under Creative Commons Attribution-Share Alike (CC BY-SA)

  This file is part of the Sinric Pro (https://github.com/sinricpro/)
 **********************************************************************************************************************************/

// STM32 Boards supported: Nucleo-144, Nucleo-64, Nucleo-32, Discovery, STM32F1, STM32F3, STM32F4, STM32H7, STM32L0, etc.
// SAM DUE
// Teensy 4.1, 4.0, 3.6, 3.5, 3.2/3.1, 3.0

#include "defines.h"

#include "SinricPro_Generic.h"
#include "SinricProSpeaker.h"

enum speakerModes 
{
  mode_movie,
  mode_music,
  mode_night,
  mode_sport,
  mode_tv
};

// we use a struct to store all states and values for our speaker
struct 
{
  bool power;
  unsigned int volume;
  bool muted;
  speakerModes mode;
  int bands[3] = { 0, 0, 0 };
} speakerState;

// Speaker device callbacks
bool onPowerState(const String &deviceId, bool &state) 
{
  Serial.println("Speaker turned " + String(state ? "on" : "off"));
  
  speakerState.power = state; // set powerState
  return true;
}

bool onSetVolume(const String &deviceId, int &volume) 
{
  Serial.println("Volume set to: " + String(volume));
  
  speakerState.volume = volume; // update Volume
  return true;
}

bool onAdjustVolume(const String &deviceId, int &volumeDelta, bool volumeDefault) 
{
  speakerState.volume += volumeDelta;   // calcualte new absolute volume
  
  Serial.println("Volume changed about " + String(volumeDelta) + " to " + String(speakerState.volume));
  
  volumeDelta = speakerState.volume;    // return new absolute volume
  
  return true;
}

bool onMute(const String &deviceId, bool &mute) 
{
  Serial.println("Speaker is " + String(mute ? "muted" : "unmuted"));
  
  speakerState.muted = mute; // update muted state
  return true;
}

bool onMediaControl(const String &deviceId, String &control) 
{
  Serial.println("MediaControl: " + control);
  
  if (control == "Play") {}           // do whatever you want to do here
  if (control == "Pause") {}          // do whatever you want to do here
  if (control == "Stop") {}           // do whatever you want to do here
  if (control == "StartOver") {}      // do whatever you want to do here
  if (control == "Previous") {}       // do whatever you want to do here
  if (control == "Next") {}           // do whatever you want to do here
  if (control == "Rewind") {}         // do whatever you want to do here
  if (control == "FastForward") {}    // do whatever you want to do here
  return true;
}

bool onSelectInput(const String &deviceId, String &input) 
{
  Serial.println("Input changed to " + input);
  return true;
}

bool onSetMode(const String &deviceId, String &mode) 
{
  Serial.println("Speaker mode set to " + mode);
  
  if (mode == "MOVIE") speakerState.mode = mode_movie;
  if (mode == "MUSIC") speakerState.mode = mode_music;
  if (mode == "NIGHT") speakerState.mode = mode_night;
  if (mode == "SPORT") speakerState.mode = mode_sport;
  if (mode == "TV") speakerState.mode = mode_tv;
  return true;
}

bool onSetBands(const String& deviceId, const String &bands, int &level) 
{
  Serial.println("Device " + deviceId + " bands " + bands + " set to " + String(level));
  
  int index;
  
  if (bands == "BASS") index = BANDS_INDEX_BASS;
  if (bands == "MIDRANGE") index = BANDS_INDEX_MIDRANGE;
  if (bands == "TREBBLE") index = BANDS_INDEX_TREBBLE;
  
  speakerState.bands[index] = level;
  return true;
}

bool onAdjustBands(const String &deviceId, const String &bands, int &levelDelta) 
{
  int index;
  
  if (bands == "BASS") index = BANDS_INDEX_BASS;
  if (bands == "MIDRANGE") index = BANDS_INDEX_MIDRANGE;
  if (bands == "TREBBLE") index = BANDS_INDEX_TREBBLE;
  speakerState.bands[index] += levelDelta;
  levelDelta = speakerState.bands[index]; // return absolute trebble level

  Serial.println("Device " + deviceId + " bands \"" + bands + "\" adjusted about " + String(levelDelta) + " to " + String(speakerState.bands[index]));
  
  return true; // request handled properly
}

bool onResetBands(const String &deviceId, const String &bands, int &level) 
{
  int index;
  
  if (bands == "BASS") index = BANDS_INDEX_BASS;
  if (bands == "MIDRANGE") index = BANDS_INDEX_MIDRANGE;
  if (bands == "TREBBLE") index = BANDS_INDEX_TREBBLE;
  speakerState.bands[index] = 0;
  level = speakerState.bands[index]; // return new level

  Serial.println("Device " + deviceId + " bands \"" + bands + "\" reset to " + String(speakerState.bands[index]));
  
  return true; // request handled properly
}


// setup function for setupEthernet connection
void setupEthernet() 
{
  #if USE_ETHERNET
    LOGWARN(F("=========== USE_ETHERNET ==========="));
  #elif USE_ETHERNET2
    LOGWARN(F("=========== USE_ETHERNET2 ==========="));
  #elif USE_ETHERNET3
    LOGWARN(F("=========== USE_ETHERNET3 ==========="));
  #elif USE_ETHERNET_LARGE
    LOGWARN(F("=========== USE_ETHERNET_LARGE ==========="));
  #elif USE_ETHERNET_ESP8266
    LOGWARN(F("=========== USE_ETHERNET_ESP8266 ==========="));
  #else
    LOGWARN(F("========================="));
  #endif
 
  LOGWARN(F("Default SPI pinout:"));
  LOGWARN1(F("MOSI:"), MOSI);
  LOGWARN1(F("MISO:"), MISO);
  LOGWARN1(F("SCK:"),  SCK);
  LOGWARN1(F("SS:"),   SS);
  LOGWARN(F("========================="));
   
  // unknown board, do nothing, use default SS = 10
  #ifndef USE_THIS_SS_PIN
    #define USE_THIS_SS_PIN   10    // For other boards
  #endif
       
  LOGWARN1(F("Use default CS/SS pin : "), USE_THIS_SS_PIN);

  // For other boards, to change if necessary
  #if ( USE_ETHERNET || USE_ETHERNET_LARGE || USE_ETHERNET2 )
    // Must use library patch for Ethernet, Ethernet2, EthernetLarge libraries

    Ethernet.init (USE_THIS_SS_PIN);

  #elif USE_ETHERNET3
    // Use  MAX_SOCK_NUM = 4 for 4K, 2 for 8K, 1 for 16K RX/TX buffer
    #ifndef ETHERNET3_MAX_SOCK_NUM
      #define ETHERNET3_MAX_SOCK_NUM      4
    #endif
    
    Ethernet.setCsPin (USE_THIS_SS_PIN);
    Ethernet.init (ETHERNET3_MAX_SOCK_NUM);
                    
  #endif  //( USE_ETHERNET || USE_ETHERNET2 || USE_ETHERNET3 || USE_ETHERNET_LARGE )

    // start the ethernet connection and the server:
  // Use Static IP
  //Ethernet.begin(mac, ip);
  // Use DHCP dynamic IP and random mac
  uint16_t index = millis() % NUMBER_OF_MAC;

  Serial.print("Index = ");
  Serial.println(index);

  Ethernet.begin(mac[index]);
  
  Serial.print("Connected!\n[Ethernet]: IP-Address is ");
  Serial.println(Ethernet.localIP());
}

// setup function for SinricPro
void setupSinricPro() 
{
  // add device to SinricPro
  SinricProSpeaker& speaker = SinricPro[SPEAKER_ID];

  // set callback functions to device
  speaker.onPowerState(onPowerState);
  speaker.onSetVolume(onSetVolume);
  speaker.onAdjustVolume(onAdjustVolume);
  speaker.onMute(onMute);
  speaker.onSetBands(onSetBands);
  speaker.onAdjustBands(onAdjustBands);
  speaker.onResetBands(onResetBands);
  speaker.onSetMode(onSetMode);
  speaker.onMediaControl(onMediaControl);
  speaker.onSelectInput(onSelectInput);

  // setup SinricPro
  SinricPro.onConnected([]()
  {
    Serial.println("Connected to SinricPro");
  });

  SinricPro.onDisconnected([]()
  {
    Serial.println("Disconnected from SinricPro");
  });

  SinricPro.begin(APP_KEY, APP_SECRET);
}

// main setup function
void setup() 
{
  Serial.begin(BAUD_RATE);
  while (!Serial);

  Serial.println("\nStarting nRF52_Ethernet_Speaker on " + String(BOARD_NAME));

  setupEthernet();
  setupSinricPro();
}

void loop() 
{
  SinricPro.handle();
}
