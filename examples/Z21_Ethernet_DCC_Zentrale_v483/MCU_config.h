//--------------------------------------------------------------
/*
 * Setup up PIN-Configuration for different MCU
 * 
 * Support for:
 *    - Arduino DUE
 *    - Arduino ESP8266
 *    - Arduino UNO
 *    - Arduino MEGA  
 *    - Sanguino (ATmgega 644p & ATmega 1284p)
 * 
 * Copyright (c) by Philipp Gahtow, year 2018
*/

#if defined(__SAM3X8E__)    //SAM ARM Adruino DUE
#define DUE_MCU
#undef RAILCOMI2C
#undef Z21VIRTUAL

#elif defined(ARDUINO_ESP8266_ESP01) || defined(ARDUINO_ESP8266_WEMOS_D1MINI) //ESP8266 or WeMos
#define ESP_MCU
#define ESP_WIFI
#undef RAILCOMI2C
#undef Z21VIRTUAL
#undef LAN
#undef DHCP

#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) //Arduino MEGA
#define MEGA_MCU
#undef RAILCOMI2C
#undef Z21VIRTUAL

#elif defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644P__)  //Sanguino (other pins!)
#define SANGUINO_MCU
//ACHTUNG SS is on PIN3 (D2)!!!
#undef Z21VIRTUAL

#else //others Arduino UNO
#define UNO_MCU

#endif

//--------------------------------------------------------------
//Z21 Button (Reset & Go/Stop):
#if defined(SANGUINO_MCU)
#define Z21ResetPin 27  //RESET-Button-Pin bei Neustart betätigen um Standard IP zu setzten!
#define Z21ButtonPin Z21ResetPin  //Pin where the POWER-Button is conected

#elif defined(UNO_MCU)
#define Z21ResetPin 10  //RESET-Button-Pin bei Neustart betätigen um Standard IP zu setzten!
#define Z21ButtonPin Z21ResetPin  //Pin where the POWER-Button is conected

#elif defined(ESP_MCU)
#define Z21ResetPin 0  //RESET-Button-Pin bei Neustart betätigen um Standard IP zu setzten!
#define Z21ButtonPin Z21ResetPin  //Pin where the POWER-Button is conected

#else //other MCU
#define Z21ResetPin 47  //RESET-Button-Pin bei Neustart betätigen um Standard IP zu setzten!
#define Z21ButtonPin Z21ResetPin  //Pin where the POWER-Button is conected
#endif

//--------------------------------------------------------------
//DCC Master & Booster:
#if defined(SANGUINO_MCU)
#define DCCLed 25    //LED to show DCC active
#define DCCPin 12    //Pin for DCC sginal out
#define ShortLed 26     //LED to show Short
#define ShortExtPin 4  //Pin to detect Short Circuit of Booster (detect LOW)
#define GoExtPin 3   //Pin for GO/STOP Signal of Booster
#define ProgRelaisPin  23   //Pin for using Kehrschleifen-Modul
//Booster INT config:
#define GoIntPin 17   //Pin for second Booster like TLE5205
#define ShortIntPin 13  //Pin for second Booster like TLE5205 (detect HIGH)
#define VAmpIntPin A4   //Input for Current sensor (CV read)

#elif defined(ESP_MCU)
#define DCCLed 15    //LED to show DCC active
#define DCCPin 12    //Pin for DCC sginal out
#define ShortLed 99     //LED to show Short
#define ShortExtPin 13  //Pin to detect Short Circuit of Booster (detect LOW)
#define GoExtPin  99   //Pin for GO/STOP Signal of Booster
#define ProgRelaisPin  99   //Pin for using Kehrschleifen-Modul
//Booster INT config:
#define GoIntPin 14   //Pin for second Booster like TLE5205
#define ShortIntPin 16  //Pin for second Booster like TLE5205 (detect HIGH)
#define VAmpIntPin A0   //Input for Current sensor (CV read)

#else //other MCU
#define DCCLed 3    //LED to show DCC active
#define DCCPin 6    //Pin for DCC sginal out
#define additionalOutPin 11 //Pin for true DCC Output without Shutdown adn RailCom
#define ShortLed 45     //LED to show Short
#define ShortExtPin 5  //Pin to detect Short Circuit of Booster (detect LOW)
#define GoExtPin  A4   //Pin for GO/STOP Signal of Booster
#define ProgRelaisPin  A5   //Pin for using Kehrschleifen-Modul
//Booster INT config:
#if defined(UNO_MCU)
#define GoIntPin 4   //Pin for second Booster like TLE5205
#define ShortIntPin 2  //Pin for second Booster like TLE5205 (detect HIGH)
#define VAmpIntPin A4   //Input for Current sensor
#else   //MEGA or DUE:
#define GoIntPin 39   //Pin for second Booster like TLE5205
#define ShortIntPin 41  //Pin for second Booster like TLE5205 (detect HIGH)
#define VAmpIntPin A9   //Input for Current sensor (CV read)
#define VAmSencePin A8  //AC 5A Sensor (for testing only)
#define VoltIntPin A10  //Rail Voltage: Rail:100k - Sence - 4,7k - GND
#define TempPin A11     //Temp.sence_resistor (15k) with 46k Pull-Up
#endif

#endif

//--------------------------------------------------------------
//Dallas Temperatur Sensor:
#if defined(DALLASTEMPSENSE) && defined(MEGA_MCU)
#define ONE_WIRE_BUS 2
#endif

//--------------------------------------------------------------
//S88 Singel Bus:
#if defined(ESP_MCU)
#undef S88N
#endif

#if defined(S88N)
  //Eingänge:
#define S88DataPin A0      //S88 Data IN
  //Ausgänge:
#define S88ClkPin A1      //S88 Clock
#define S88PSPin A2       //S88 PS/LOAD
#define S88ResetPin A3    //S88 Reset
#endif
//--------------------------------------------------------------
//DCC Decoder
#if defined(SANGUINO_MCU)
#undef DECODER
#endif
#if defined(ESP_MCU)
#undef DECODER
#endif

   //Eingänge:
#if defined(DECODER)
#define IRQDCCPin 0      //Arduino Interrupt Number (attachInterrupt Funktion)
#define decDCCPin 2      //The Digital PIN where the Interrupt is on
#endif

//--------------------------------------------------------------
//XpressNet-Bus:
#if defined(SANGUINO_MCU)
#define XNetTxRxPin  16    //XpressNet Control-Port for Send/Receive at MAX485

#elif defined(ESP_MCU)
#define XNetTxRxPin 5  //XpressNet Control-Port for Send/Receive at MAX485
  
#else //other MCU
#define XNetTxRxPin  9    //XpressNet Control-Port for Send/Receive at MAX485
//#define XNetTxRxPin  44    //Extern Shield
#endif

//--------------------------------------------------------------
//LocoNet-Bus:
#if defined(ESP_MCU)
#undef LOCONET
#endif

#if defined(SANGUINO_MCU)
#define LNTxPin 15    //Sending Pin for LocoNet
  
#else //other MCU
#define LNTxPin 7    //Sending Pin for LocoNet
//#define LNTxPin 46    //Extern Shield
#endif

//--------------------------------------------------------------
//Wifi-Interface:
#if defined(ESP_MCU)
#undef WIFI
#endif

#if defined (WIFI)
//Serialport:
#ifndef WLAN  //WLAN defined
#if defined(MEGA_MCU) || defined(DUE_MCU) //MCU check: Arduino MEGA
#define WLAN Serial2
#elif defined(Z21VIRTUAL)
#define WLAN SoftSerial
#else
#define WLAN Serial
#endif  //END MCU check
#if defined(Z21VIRTUAL) //Default Serial Baud Rate 1200,2400,4800,9600,14400,19200,28800,38400,57600,115200 
#define WIFISerialBaud 38400
#else
#define WIFISerialBaud 500000
#endif //END Z21VIRTUAL
#endif  //END WLAN defined

#if defined(Z21VIRTUAL)   //for Arduino UNO only:
#define RXvWiFi 12  //RX-PIN Soft Serial for Arduino UNO WiFi use  
#define TXvWiFi 11  //TX-PIN Soft Serial for Arduino UNO WiFi use  
#endif

#endif

