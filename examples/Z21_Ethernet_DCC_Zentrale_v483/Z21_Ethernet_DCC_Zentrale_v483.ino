/**************************************************************
 * Z21 Ethernet DCC Command Station
 * Copyright (c) 2015-2019 by Philipp Gahtow
***************************************************************
Unterstützte Funktionen/Protokolle:
 * NMRA DCC output (with Railcom and POM)
 * Z21 Ethernet over LAN and/or WLAN
 * S88N feedback
 * XpressNet with AUTO switch MASTER-SLAVE-MODE
 * LocoNet with MASTER-MODE (Slotserver) or SLAVE-MODE
 * NMRA DCC Input (legacy)
 * support for ATmega 328/644/1280/1284/2560 (UNO, MEGA, Sanguino)
 * support for ARM (DUE)
 * --> all functions/interfaces only awayable on Arduino MEGA / DUE

This is a simple dcc command station that receive commands via Ethernet, XpressNet or LocoNet.
It base on the Z21 ethernet protocol of ROCO!

***************************************************************

- DCC Master Interface with Timer 2 by modifired CmdrArduino library by Philipp Gahtow
- Z21 LAN Protokoll mit W5100 Ethernet Shield with z21.h library
- LAN HTTP Website on Port 80 to configure ethernet IP and S88 bus length
- ESP8266 WiFi Z21 LAN Untersützung with z21.h library
- fast S88N feedback
- legacy:(DCC input, to read signal from other Central Station via Interrupt 0 and Timer 4)
- LocoNet at MEGA with Timer 5, normal Timer1 with Loconet.h library
- XpressNet (RS485) via LOOP-Function with XpressNetMaster.h library
- Relais for seperate program track
- Global Railcom Detector for MEGA on Serial3

***************************************************************

*Softwareversion: */
#define Z21mobileVERSIONMSB 4
#define Z21mobileVERSIONLSB 83
/*
---------------------------------------------------------------
changes:
15.04.2015  Abschaltung S88 Interface per Define (S88N)
16.04.2015  Aktualisierung Z21 LAN Protokoll V1.05 & Firmware-Version 1.26
17.04.2015  LN OPC_INPUT_REP msg von Belegmeldern über LAN_LOCONET_DETECTOR
20.04.2015  kurze/Lange DCC Adressen (1..99 kurz, ab 100 lang)
22.04.2015  Add in DCC Lib Function support F13 - F28
            Add Power Button with Reset (press at startup)
23.04.2015  Add LocoNet set Railpower (OPC_GPOFF, OPC_GPON, OPC_IDLE)
            Add LocoNet Slot Write (OPC_WR_SL_DATA)
            New Broadcast Msg (8 Bit) Z21 Protokoll V1.05 (Include LocoNet)
            Add LocoNet OPC_RQ_SL_DATA, OPC_UHLI_FUN, OPC_SW_REQ, OPC_SW_REP, OPC_SW_ACK, OPC_SW_STATE
28.04.2015  Add DCC CV Write and Decoder Reset Packet before CV-Programming            
04.07.2015  Add Support Sanguino (ATmega644p and ATmega1284p) =>MCU_config.h
10.07.2015  Change Timer for DCC Interface and S88 to support LocoNet for all MCU
            Add second Booster support (intenal/external)
21.07.2015  S88 max Module define hinzu und S88 HTTP Ausgabe angepasst
30.07.2015  Versionsnummer für Debug definiert
02.08.2015  DCC Accessory Befehl korrigiert
            PowerButton Input geändert von Pin 51 nach Pin 47
03.08.2015  DCC Decoder Funktion korrigiert
17.09.2015  S88 Timer Auswahl (MEGA = Timer3)
18.09.2015  ESP8266 WiFi Support; Z21 LAN über externe Library
23.09.2015  Überarbeitung LAN_LOCONET_DETECTOR
            Neues Kommando OPC_MULTI_SENSE
            DCC Dekoder ohne Timer4!
            Optionale Lok-Event-Informationen im LocoNet (reduzierung der Sendedaten)
03.10.2015  S88 verbessert -> Fehler in der S88 Modulanzahl korrigiert (Überlauf der Zählervariale)       
            LocoNet TX/RX Packetverarbeitung verbessert  
04.10.2015  ROCO EXT Booster Short mit Transistor (invertiert!) 
            Optimierung S88 Timer (Rechenoperationen und Seicherbedarf)              
10.10.2015  Anzeigen Reset Zentrale mittels binkenden LEDs   
13.10.2015  Rückmelder über LocoNet
            Senden von DCC Weichenschaltmeldungen auch über LocoNet         
            LAN Webseite angepasst für Smartphone Display
14.10.2015  Einstellung der Anzahl von S88 Modulen über WiFi
            Verbesserung der Kommunikation mit dem ESP    
04.11.2015  LocoNet Master- oder Slave-Mode auswählbar
19.12.2015  Support kombinierte UDP Paket für WLAN und LAN            
26.12.2015  Add Track-Power-Off after Service Mode 
20.02.2016  Speicherreduzierung wenn kein WLAN und LAN genutzt wird
            LocoNet Client Modus Kommunikation mit IB verbessert
            Extra Serial Debug Option für XpressNet
27.02.2016  Änderung Dekodierung DCC14 und DCC28
            Invertierung Fahrtrichtung DCC Decoder DIRF            
            LocoNet Slave-Mode ignoriere Steuerbefehle, wenn Slot = 0
02.06.2016  Baud für Debug und WiFi einstellbar
            Software Serial für WiFi wählbar (zB. für Arduino UNO)
            -> WiFi Modul Firmware ab v2.5
17.07.2016 Fix Network UDP Antwortport - Sende Pakete an Quellport zurück
25.07.2016 add busy message for XpressNet (MultiMaus update screen now)
Aug.2016   add Railcom Support and update DCCInterfaceMaster and Booster Hardware,
           support POM read over I2C with external MCU (GLOBALDETECTOR)
26.08.2016 add DHCP for Ethernet Shield      
21.11.2016 DCC: fix Railcom - still Problem with Startup: Analog-Power on the rails - Hardware change needed!
26.11.2016 LocoNet: add Uhlenbrock Intellibox-II F13 to F28 support
27.11.2016 LocoNet: fix Speed DIR in OPC_SL_RD_DATA in data byte to 0x80 = B10000000 and OPC_LOCO_DIRF remove invert
27.12.2016 Z21 add CV lesen am Programmiergleis
01.02.2017 add negative DCC output option and seperate this feature from RAILCOM
15.03.2017 fix narrowing conversation inside LNInterface.h
28.03.2017 external Booster active in ServiceMode when no internal Booster
24.04.2017 fix data lost on loconet - s88 timer3 block packets - deactivated
28.04.2017 add MultiMaus support for F13 to F20 without fast flashing
10.05.2017 add XpressNet information for loco speed and function and switch position change
11.05.2017 add internal Booster Short Detection over Current Sence Resistor
25.05.2017 add RailCom Global Reader for Arduino MEGA on Serial3 (POM CV read only)
19.06.2017 fix problems with Arduino UNO compiling
09.07.2017 fix problems when using without XpressNet
23.07.2017 add support for Arduino DUE
26.08.2017 add default speed step setting
09.01.2018 add POM Bit write
21.01.2018 optimize LocoNet Slot system - reduce RAM use
18.08.2018 add support for Arduino DUE XpressNet
02.11.2018 adjust Z21 WiFi data communication and rise up baud rate
22.11.2018 add support for Arduino ESP8266 (WiFi, DCC extern and intern without seperate prog track
09.06.2019 add extra DCC-Output for S88 and LocoNet without Power-OFF and RailCom
---------------------------------------------------------------
toDo:
-> Rückmelder via XpressNet
-> Programmieren von CVs im LocoNet -> works with Z21mobile APP over LocoNet Tunnel!
(-> store loco adr use via per ip? (max 16?))
-> read loco adress with global RailCom detector

*/
/*--------------------------------------------------------------------------------------------------------
-----------------------------------**********************************-------------------------------------
                                   CHANGE DOWN HERE THE CONFIGURATION
----------------------------------------------------------------------------------------------------------
Command Station Config:
=> uncomment ("//" or #undef) the following lines, if you not USE the protokoll!
*/
/**************************************************************/
#define Debug Serial  //Interface for Debugging
#define DebugBaud 115200  //Debug speed
#define DEBUG    //To see DATA on Serial
#define REPORT    //To see Sensor Messages (LocoNet & S88)
//#define LnDEB    //To see HEX DATA of LocoNet Protokoll
//#define XnDEB    //To see XpressNet
//#define Z21DEBUG //to see Z21 LAN control data
//#define Z21DATADEBUG //to see DATA of Z21 LAN Protokoll
//#define Z21SYSTEMDATADEBUG  //to see the mainpower and temp
//#define DEBUG_WLAN_CONFIG  //to see config data of Wifi ESP8266 (IP, Pw, ..)
//#define RCDEB     //To see RailCom Data

/**************************************************************
Singel S88 Bus Interface (max 62 * 8 Module)*/
#define S88N

/**************************************************************
WiFi ESP 8266 Z21 LAN Komunikation via Serial*/
#define WIFI
//#define Z21VIRTUAL  //WiFi over SoftSerial for UNO only! - LAN and LocoNet will be inaktiv!

/**************************************************************
WiFi ESP 8266 Z21 Central Station*/
//Info: #define ESP_WIFI is setting up by MCU config!!!
//#define ESP_HTTPCONF  //Website to configure IP Adress and Hotsport settings (only ESP8266 central)

/**************************************************************
LAN W5100 Ethernet Z21 LAN Kommunikation*/
#define LAN       //Standard IP ist 192.168.0.111. Bitte diese IP nur über die Webseite (http://192.168.0.111) ändern!
//#define DHCP      //Activate to Receive a IP Adress from the DHCP Server, if no DHCP found fix IP Adress vom EEPROM will be load.
#define HTTPCONF  //Website to configure IP Adress and Number of S88 Bus Module

/**************************************************************
Dallas 18B20 Temperatur Sendor */
//#define DALLASTEMPSENSE

/**************************************************************
DCC RailCom Global Detector */
//#define DCCGLOBALDETECTOR  //activate the DCC Railcom Global Detector for MEGA on Serial Port 3 (RX only)

/*RailCom For Arduino UNO (!!!in testing!!!):*/
//#define RAILCOMI2C  //I2C Reading for RailCom over external MCU (legacy: support only for UNO and Sanduino!)

/**************************************************************
DCC Decoder (only for MEGA, To decode a DCC-Signal, add this data to command station DCC output)*/
//#define DECODER   //outdated-not in use anymore!!!

/**************************************************************
XpressNet Master Interface*/
#define XPRESSNET 
#include <XpressNetMaster.h>

/**************************************************************
LocoNet Master Interface (Timer1, Timer5 on MEGA)*/
#define LOCONET  
#include <LocoNet.h>
#define TXAllLokInfoOnLN false    //sende alle Lok-Ereignisse ins LocoNet (MASTER-MODE only)
#define LnSLOTSRV    //Z21 DCC Arduino provide a Slot Server for Loco (MASTER-MODE)

/**************************************************************
Booster external: (zB. ROCO, CD[E])*/
#define BOOSTER_EXT
#define BOOSTER_EXT_ON HIGH
#define BOOSTER_EXT_OFF LOW

/**************************************************************
Booster internal: (zB. TLE5205)*/
#define BOOSTER_INT
#define BOOSTER_INT_ON LOW    //only for old Mode without RAILCOM support over NDCC!
#define BOOSTER_INT_OFF HIGH  //only for old Mode without RAILCOM support over NDCC!
/*(GoIntPin) activate inverted booster signal*/
#define BOOSTER_INT_NDCC    //for new RAILCOM Booster3R
/*(VAmpIntPin) activate the current sensor for prog track:*/
#define BOOSTER_INT_MAINCURRENT   //MEGA, SANGUINO and for UNO only without external Booster, 
/*(VAmpIntPin) activate SHORT CIRCUIT SENCE over MAINCURRENT*/
#define BOOSTER_INT_CURRENT_SHORT_DETECT     //alternativ Short Circuit Detection over current sence resistor

/**************************************************************
DCC Master to create a DCC Signal:*/
#include <DCCPacketScheduler.h>   //DCC Interface Library
//---------
#define SwitchFormat ROCO   //ROCO (+0) or IB (+4) => Define Accessory Address start value!

#define FS128   //default Fahrstufen (Speed Steps) => possible values: FS14, FS28, FS128
/***************************************************************/

//--------------------------------------------------------------------------------------------------------
//----------------------------------*********************************-------------------------------------
//                                  DON'T CHANGE ANYTHING DOWN HERE!
//--------------------------------------------------------------------------------------------------------
//Setup up PIN-Configuration for different MCU (UNO/MEGA/Sanduino)
#include "MCU_config.h"

//**************************************************************
#if defined(Z21VIRTUAL)  
#include <SoftwareSerial.h>
SoftwareSerial SoftSerial(TXvWiFi, RXvWiFi); // init Soft Serial
#undef LAN    //LAN nicht zulassen - Doppelbelegung der Signalleitungen!
#undef HTTPCONF
#undef LOCONET
#endif

#if defined(LAN)      //W5100 LAN Interface Library
#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library
#endif

#if defined(ESP_WIFI)   //ESP8266 chip
#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <WiFiUDP.h>
#endif

//**************************************************************
static void globalPower (byte state);

//Z21 LAN Protokoll:
#if defined(LAN) || defined (WIFI) || defined(ESP_WIFI)
#include <z21.h> 
z21Class z21;
#endif

//**************************************************************
#if defined(DUE_MCU)
#include <DueFlashStorage.h>
DueFlashStorage Flash;
#define FIXSTORAGE Flash
#define FIXMODE write
#else
#include <EEPROM.h>   //EEPROM - to store number of S88 Module and LAN IP
#define FIXSTORAGE EEPROM
  #if defined(ESP_MCU)
  #define FIXMODE write
  #else
  #define FIXMODE update
  #endif
#endif
#if defined(ESP_MCU)
//EEPROM Konfiguration
#define EESize 256    //Größe des EEPROM
//Client:
#define EEssidLength 0       //Länge der SSID
#define EEssidBegin 1        //Start Wert
#define EEpassLength 32        //Länge des Passwort
#define EEpassBegin 33        //Start Wert
//AP:
#define EEssidAPLength 64       //Länge der SSID AP
#define EEssidAPBegin 65        //Start Wert
#define EEpassAPLength 98        //Länge des Passwort AP
#define EEpassAPBegin 99        //Start Wert
#define EEkanalAP 150          //Kanal AP
//config:
#define EES88Moduls 160  //Adresse EEPROM Anzahl der Module für S88
#define EEip 161    //Startddress im EEPROM für die IP
#else
#define EES88Moduls 38  //Adresse EEPROM Anzahl der Module für S88
#define EEip 40    //Startddress im EEPROM für die IP
#endif

//---------------------------------------------------------------
#if defined(LAN)  //W5100 LAN Udp Setup:
EthernetUDP Udp;    //UDP for communication with APP/Computer (Port 21105)
//EthernetUDP UdpMT;  //UDP to Z21 Maintenance Tool (Port 34472)
//---------------------------------------------------------------
// The IP address will be dependent on your local network:
// Die MAC Adresse der Z21 beginnt mit „84:2B:BC:..:..:..“!
static byte mac[] = { 0x84, 0x2B, 0xBC, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 0, 111);   //Werkseinstellung ist: 192.168.0.111

#if defined(HTTPCONF) //W5100 LAN config Website:
EthernetServer server(80);  // (port 80 is default for HTTP):
#endif
#endif    //LAN end
//---------------------------------------------------------------
#if defined(ESP_WIFI)
#define SssidAP "Z21_ESP_Central"   // Default Z21 AP (SSID)
#define SpassAP "12345678"  // Default Z21 network password
#define SkanalAP 3          // Default Kanal des AP

WiFiUDP Udp;
#if defined(ESP_HTTPCONF) //Setting Website
WiFiServer server(80);  //default port 80 for HTTP
#endif
#endif
//--------------------------------------------------------------
//Z21 Protokoll Typ Spezifikationen
#if defined(LAN) || defined (WIFI) || defined(ESP_WIFI)
#include "Z21type.h"    //Z21 Data Information
#endif

//--------------------------------------------------------------
//S88 Singel Bus:
#if defined(S88N)
#include "S88.h"
#endif

//--------------------------------------------------------------
//Dallas Temperatur Sensor:
#if defined(DALLASTEMPSENSE) && defined(MEGA_MCU)
#include <OneWire.h>
#include <DallasTemperature.h>
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
// arrays to hold device address
DeviceAddress insideThermometer;
#endif

//--------------------------------------------------------------
//DCC Interface Master Short Detection:
//EXTERNAL BOOSTER:
#define DetectShortCircuit 0x1FF    //to detect short circuit  (0xFF)
unsigned int ShortTime = 0;            //Time Count for Short Detect
unsigned long LEDcount = 0;    //Timer for Status LED
//INTERNAL BOOSTER:
#if defined(BOOSTER_INT_CURRENT_SHORT_DETECT)
byte ShortTimeINT = 0;      //Time Count for internal short detect
#define DetectShortCircuit_INT  3 //Time after internal short circuit is detected
#define DETECT_SHORT_INT_VALUE  400  //analogRead value for "mA" that is too much
#endif

//--------------------------------------------------------------
DCCPacketScheduler dcc;
#define DCC     //activate DCC Interface

//--------------------------------------------------------------
#if defined(DCCGLOBALDETECTOR) && defined(DCC)
#include "Z21_RailCom.h"

#if defined(RAILCOMI2C) && defined(DCCGLOBALDETECTOR)
#include <Wire.h>
#endif

#endif

//--------------------------------------------------------------
#if defined(XPRESSNET)
XpressNetMasterClass XpressNet;
  #ifdef __SAM3X8E__
  #include "XpressNet_DUE.h"
  #endif
#endif

//--------------------------------------------------------------
// certain global XPressnet status indicators
#define csNormal 0x00 // Normal Operation Resumed ist eingeschaltet
#define csEmergencyStop 0x01 // Der Nothalt ist eingeschaltet
#define csTrackVoltageOff 0x02 // Die Gleisspannung ist abgeschaltet
#define csShortCircuit 0x04 // Kurzschluss
#define csServiceMode 0x08 // Der Programmiermodus ist aktiv - Service Mode
byte Railpower = csTrackVoltageOff;   //State of RailPower at Startup
bool Z21ButtonLastState = false;    //store last value of the Push Button for GO/STOP

//--------------------------------------------------------------
//LocoNet-Bus:
#if defined (LOCONET)
#include "LNInterface.h"
#endif

//--------------------------------------------------------------
//DCC Decoder:
#if defined(DECODER)
#include "DCCDecoder.h"
#endif

//--------------------------------------------------------------
//XpressNet-Bus:
#if defined(XPRESSNET)
#include "XBusInterface.h"
#endif

//--------------------------------------------------------------
//Z21 Ethernet communication:
#if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
#include "Z21_LAN.h"
#endif

//--------------------------------------------------------------------------------------------
//POWER set configuration:
static void globalPower (byte state) {
  if (Railpower != state) {
    
    #if defined(LAN) || defined (WIFI)  
    if (Railpower == csServiceMode && state == csShortCircuit) {
      z21.setCVNackSC();  //response SHORT while Service Mode!
    }
    #endif
    
    Railpower = state;
    #if defined(DEBUG)
    Debug.print(F("Power: "));
    Debug.println(state);
    #endif
    switch (state) {
      case csNormal: 
        #if defined(DCC)
        dcc.setpower(ON);
        digitalWrite(ProgRelaisPin, LOW);     //ProgTrack 
        #endif
        #if defined(BOOSTER_EXT)
        if (digitalRead(ShortExtPin) == LOW)
          digitalWrite(GoExtPin, BOOSTER_EXT_ON);
        #endif
       
        #if (defined(BOOSTER_INT) && !defined(BOOSTER_INT_NDCC))
        digitalWrite(GoIntPin, BOOSTER_INT_ON);
        #endif
     
      break;
      case csTrackVoltageOff: 
        #if defined(DCC)
        dcc.setpower(OFF);
        digitalWrite(ProgRelaisPin, LOW);     //ProgTrack 
        #endif
        #if defined(BOOSTER_EXT)
        digitalWrite(GoExtPin, BOOSTER_EXT_OFF);
        #endif
        
        #if (defined(BOOSTER_INT) && !defined(BOOSTER_INT_NDCC))
        digitalWrite(GoIntPin, BOOSTER_INT_OFF);
        #endif
        
      break;
      case csServiceMode:
        #if defined(DCC) 
        dcc.setpower(SERVICE); //already on!
        digitalWrite(ProgRelaisPin, HIGH);     //ProgTrack 
        #endif
        #if defined(BOOSTER_EXT)
          #if defined(BOOSTER_INT)
          digitalWrite(GoExtPin, BOOSTER_EXT_OFF);
          #else
          if (digitalRead(ShortExtPin) == LOW)
            digitalWrite(GoExtPin, BOOSTER_EXT_ON);
          #endif
        #endif

        #if (defined(BOOSTER_INT) && !defined(BOOSTER_INT_NDCC))
        digitalWrite(GoIntPin, BOOSTER_INT_ON);
        #endif
        
      break;
      case csShortCircuit: 
        #if defined(DCC)
        dcc.setpower(SHORT);  //shut down via GO/STOP just for the Roco Booster
        digitalWrite(ProgRelaisPin, LOW);     //ProgTrack 
        #endif
        #if defined(BOOSTER_EXT)
        digitalWrite(GoExtPin, BOOSTER_EXT_OFF);
        #endif
        
        #if (defined(BOOSTER_INT) && !defined(BOOSTER_INT_NDCC))
        digitalWrite(GoIntPin, BOOSTER_INT_OFF);
        #endif
        
      break;
      case csEmergencyStop:
        #if defined(DCC)
        dcc.eStop();  
        #endif
      break;
    }
    if (Railpower == csShortCircuit)
      digitalWrite(ShortLed, HIGH);   //Short LED show State "short"
    if (Railpower == csNormal)  
      digitalWrite(ShortLed, LOW);   //Short LED show State "normal" 
    #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
    z21.setPower(Railpower);
    #endif
    #if defined(XPRESSNET)
    XpressNet.setPower(Railpower);  //send to XpressNet
    #endif
    #if defined(LOCONET)
    LNsetpower(); //send to LocoNet
    #endif
  }
}

//--------------------------------------------------------------------------------------------
//from DCCPacketScheduler -> notify power state
void notifyPower(uint8_t state) {
  globalPower(state);
}

//--------------------------------------------------------------------------------------------
#if defined(DCC)
void ShortDetection() { 
  //Short Circuit?
  //Check BOOSTER extern
  #if defined(BOOSTER_EXT)
  if ((digitalRead(ShortExtPin) == HIGH) && (digitalRead(GoExtPin) == BOOSTER_EXT_ON) && (Railpower != csShortCircuit)) {  
    ShortTime++;
    if(ShortTime == DetectShortCircuit) {
        globalPower(csShortCircuit);
        #if defined(DEBUG)
        Debug.println(F("TRACK_SHORT_CIRCUIT EXT"));
        #endif
    }
    /*  NOT IN USE ANYMORE from v4.75 on!
    //Before Railpower cut out test change polarity:
    else if (ShortTime == KSRelaisShortCircuit) {   
      digitalWrite(KSPin, !digitalRead(KSPin));     //Kehrschleife
      #if defined(DEBUG)
      Debug.print(F("KS "));
      Debug.println( digitalRead(KSPin) );
      #endif
    }
    */
  }
  else ShortTime = 0;
  #endif
  //Check BOOSTER2 (z.B. TLE5205)
  #if defined(BOOSTER_INT)
  #if defined(BOOSTER_INT_NDCC)
  if ((digitalRead(ShortIntPin) == LOW) && (Railpower != csShortCircuit)) {
  #else
  if ((digitalRead(ShortIntPin) == LOW) && (digitalRead(GoIntPin) == BOOSTER_INT_ON) && (Railpower != csShortCircuit)) {
  #endif
    globalPower(csShortCircuit);
    #if defined(DEBUG)
    Debug.println(F("TRACK_SHORT_CIRCUIT INT"));
    #endif
  }
  #if defined(BOOSTER_INT_CURRENT_SHORT_DETECT) && defined(BOOSTER_INT_MAINCURRENT)
  uint16_t VAmp = analogRead(VAmpIntPin);
  if ((VAmp >= DETECT_SHORT_INT_VALUE) && (Railpower != csShortCircuit)) {
    ShortTimeINT++;
    if (ShortTimeINT == DetectShortCircuit_INT) {
      globalPower(csShortCircuit);
      #if defined(DEBUG)
      Debug.print(VAmp);
      Debug.println(F(" TRACK_SHORT_CIRCUIT INT"));
      #endif
    }
  }
  else ShortTimeINT = 0;
  #endif
  #endif
}
#endif

//--------------------------------------------------------------------------------------------
void updateLedButton() {
  //Button to control Railpower state
  if ((digitalRead(Z21ButtonPin) == LOW) && (Z21ButtonLastState == false)) {  //Button DOWN
    Z21ButtonLastState = true;
    LEDcount = millis();
  }
  else {
    if ((digitalRead(Z21ButtonPin) == HIGH) && (Z21ButtonLastState == true)) {  //Button UP
       #if defined(DEBUG)
         Debug.print(F("Button "));
      #endif
      unsigned long currentMillis = millis(); 
      Z21ButtonLastState = false;
      if(currentMillis - LEDcount > 750) //push long?
        if (FIXSTORAGE.read(52) == 0x00)  //Power-Button (short): 0=Gleisspannung aus, 1=Nothalt  
          globalPower(csEmergencyStop);  
        else globalPower(csTrackVoltageOff);
      else {
        if (Railpower == csNormal) {
          if (FIXSTORAGE.read(52) == 0x00) //Power-Button (short): 0=Gleisspannung aus, 1=Nothalt  
            globalPower(csTrackVoltageOff);
          else globalPower(csEmergencyStop);
        }
        else globalPower(csNormal);
      }
      LEDcount = 0;
    }
  }
  //Update LED  
  if (Z21ButtonLastState == false) {  //flash
    if (Railpower == csNormal) {
      digitalWrite(DCCLed, HIGH);
      return;
    }
    unsigned long currentMillis = millis(); 
    if (currentMillis > LEDcount) {
      if (Railpower == csTrackVoltageOff) {
        if (digitalRead(DCCLed) == HIGH)
          LEDcount = currentMillis + 1100;    //long OFF
        else LEDcount = currentMillis + 300;  //short ON
      }
      if (Railpower == csEmergencyStop) {
        if (digitalRead(DCCLed) == HIGH)
          LEDcount = currentMillis + 80;    //short OFF
        else LEDcount = currentMillis + 700;  //long ON
      }
      if (Railpower == csShortCircuit) 
        LEDcount = currentMillis + 200;  //short flash
        
      digitalWrite(DCCLed, !digitalRead(DCCLed));
    }
  }
}

//--------------------------------------------------------------------------------------------
#if defined(HTTPCONF) && defined(LAN)
void Webconfig() {
  EthernetClient client = server.available();
  if (client) {
    String receivedText = String(50);
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (receivedText.length() < 50) {
          receivedText += c;
        }
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println(F("HTTP/1.1 200 OK"));
          client.println(F("Content-Type: text/html"));
          client.println(F("Connection: close"));  // the connection will be closed after completion of the response
          //client.println(F("Refresh: 5"));  // refresh the page automatically every 5 sec
          client.println();   //don't forget this!!!
          //Website:
          client.println(F("<!DOCTYPE html>"));
          client.println(F("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"/>"));
          client.println(F("<html><title>Z21</title><body><h1>Z21</h1>"));
          //----------------------------------------------------------------------------------------------------          
          int firstPos = receivedText.indexOf("?");
          if (firstPos > -1) {
            byte lastPos = receivedText.indexOf(" ", firstPos);
            String theText = receivedText.substring(firstPos+3, lastPos); // 10 is the length of "?A="
            byte S88Pos = theText.indexOf("&S88=");
            #if defined(S88N)
              S88Module = theText.substring(S88Pos+5, theText.length()).toInt();
            #endif  
            byte Aip = theText.indexOf("&B=");
            byte Bip = theText.indexOf("&C=", Aip);
            byte Cip = theText.indexOf("&D=", Bip);
            byte Dip = theText.substring(Cip+3, S88Pos).toInt();
            Cip = theText.substring(Bip+3, Cip).toInt();
            Bip = theText.substring(Aip+3, Bip).toInt();
            Aip = theText.substring(0, Aip).toInt();
            ip[0] = Aip;
            ip[1] = Bip;
            ip[2] = Cip;
            ip[3] = Dip;
            client.println(F("-> RESET Z21"));
            #if defined(S88N)
            if (FIXSTORAGE.read(EES88Moduls) != S88Module) {
              FIXSTORAGE.write(EES88Moduls, S88Module);
              SetupS88();
              #if defined(WIFI)
              WLANSetup();
              #endif
            }
            #endif
            FIXSTORAGE.FIXMODE(EEip, Aip);
            FIXSTORAGE.FIXMODE(EEip+1, Bip);
            FIXSTORAGE.FIXMODE(EEip+2, Cip);
            FIXSTORAGE.FIXMODE(EEip+3, Dip);
          }
          //----------------------------------------------------------------------------------------------------          
          client.print(F("<form method=get>IP:<input type=number min=0 max=254 name=A value="));
          client.println(ip[0]);
          #if defined(DHCP)
          client.print(F(" disabled=disabled"));
          #endif
          client.print(F("><input type=number min=0 max=254 name=B value="));
          client.println(ip[1]);
          #if defined(DHCP)
          client.print(F(" disabled=disabled"));
          #endif
          client.print(F("><input type=number min=0 max=254 name=C value="));
          client.println(ip[2]);
          #if defined(DHCP)
          client.print(F(" disabled=disabled"));
          #endif
          client.print(F("><input type=number min=0 max=254 name=D value="));
          client.println(ip[3]);
          #if defined(DHCP)
          client.print(F(" disabled=disabled"));
          #endif
          client.print(F("><br/>8x S88:<input type=number min=0 max="));
          #if defined(S88N)
          client.print(S88MAXMODULE);
          #else
          client.print("0");
          #endif
          client.print(F(" name=S88 value="));
          #if defined(S88N)
            client.print(S88Module);
          #else
            client.print("-");
          #endif
          client.println(F("><br/><input type=submit></form></body></html>"));
          break;
        }
        if (c == '\n') 
          currentLineIsBlank = true; // you're starting a new line
        else if (c != '\r') 
          currentLineIsBlank = false; // you've gotten a character on the current line
      }
    }
    client.stop();  // close the connection:
  }
}
#endif

//--------------------------------------------------------------------------------------------
#if defined(ESP_MCU) && defined(ESP_HTTPCONF)
void Webconfig() {
  WiFiClient client = server.available();
  if (!client)
    return;
    
  String HTTP_req;            // stores the HTTP request 

  if (client) {  // got client?
        boolean currentLineIsBlank = true;
        while (client.connected()) {
            if (client.available()) {   // client data available to read
                char c = client.read(); // read 1 byte (character) from client
                HTTP_req += c;  // save the HTTP request 1 char at a time
                // last line of client request is blank and ends with \n
                // respond to client only after last line received
                if (c == '\n' && currentLineIsBlank) {
                    // send a standard http response header
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: text/html");
                    client.println("Connection: keep-alive");
                    client.println();      //don't forget this!!!
                    // AJAX request for switch state
                    if (HTTP_req.indexOf("/ajax_switch") > -1) {
                        // read switch state and send appropriate paragraph text
                        ssid = HTTP_req.substring(HTTP_req.indexOf("&s=")+3,HTTP_req.indexOf("&p="));
                        pass = HTTP_req.substring(HTTP_req.indexOf("&p=")+3,HTTP_req.indexOf("&As="));
                        ssidAP = HTTP_req.substring(HTTP_req.indexOf("&As=")+4,HTTP_req.indexOf("&Ap="));
                        passAP = HTTP_req.substring(HTTP_req.indexOf("&Ap=")+4,HTTP_req.indexOf("&Ak="));
                        kanalAP = HTTP_req.substring(HTTP_req.indexOf("&Ak=")+4,HTTP_req.indexOf("&S8=")).toInt();
                        #if defined(S88N)                        
                          S88Module = HTTP_req.substring(HTTP_req.indexOf("&S8=")+4,HTTP_req.indexOf("&nocache")).toInt();
                        #endif
                        
                        if ((kanalAP < 1) || (kanalAP > 13)) {
                          kanalAP = SkanalAP;
                          client.print("Ka. error! ");
                        }
                        if (passAP.length() < 8) {
                          passAP = SpassAP;
                          client.print("Code length error (min. 8)! ");
                        }
                        
                        // write eeprom
                        EEPROMwrite (ssid, EEssidLength, EEssidBegin);
                        EEPROMwrite (pass, EEpassLength, EEpassBegin);
                        
                        EEPROMwrite (ssidAP, EEssidAPLength, EEssidAPBegin);
                        EEPROMwrite (passAP, EEpassAPLength, EEpassAPBegin);
                        EEPROM.write(EEkanalAP, kanalAP);
                        EEPROM.commit(); 

                        WiFi.disconnect();
                        tryWifiClient();
                        
                        Udp.begin(z21Port);

                        client.println("saved");   //OK!
                    }
                    else {  // HTTP request for web page
                        // send web page - contains JavaScript with AJAX calls
                        client.println("<!DOCTYPE html>");
                        client.println("<html><head><title>Z21</title>");
                        client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"/>");
                        client.println("<script>");
                        client.println("function SetState() {");
                        client.println("document.getElementById(\"state\").innerHTML = \"wait\";");
                        client.println("nocache = \"&s=\" + document.getElementById(\"ssid\").value;");
                        client.println("nocache += \"&p=\" + document.getElementById(\"pass\").value;");
                        client.println("nocache += \"&As=\" + document.getElementById(\"ssidAP\").value;");
                        client.println("nocache += \"&Ap=\" + document.getElementById(\"passAP\").value;");
                        client.println("nocache += \"&Ak=\" + document.getElementById(\"kanalAP\").value;");
                        client.println("nocache += \"&S8=\" + document.getElementById(\"S88\").value;");
                        client.println("nocache += \"&nocache=\" + Math.random() * 1000000;");
                        client.println("var request = new XMLHttpRequest();");
                        client.println("request.onreadystatechange = function() {");
                        client.println("if (this.readyState == 4){");
//                        client.println("if (this.status == 200){");
//                        client.println("if (this.responseText != null) {");
                        client.println("document.getElementById(\"state\").innerHTML = this.responseText;");
                        client.println("top.window.location.reload(true);");
                        client.println("}}");
                        client.println("request.open(\"GET\", \"ajax_switch\" + nocache, true);");
                        client.println("request.send(null);");
                        //client.println("setTimeout('SetState()', 1000);");
                        client.println("}");
                        client.println("</script>");
                        client.println("</head>");
                        client.println("<body><h1>Z21 Net-config</h1><hr>");
                        client.print("<h2>WiFi Direct AP</h2>");
                        client.print("<dl><dd>IP: ");
                        client.print(WiFi.softAPIP());
                        client.print("</dd><dd>Connected Clients: ");
                        client.print(WiFi.softAPgetStationNum());
                        client.print(" of 4</dd><dd>SSID: <input type=\"text\" id=\"ssidAP\" value=\"");
                        client.print(ssidAP);
                        client.print("\"></dd><dd>code: <input type=\"text\" id=\"passAP\" value=\"");
                        client.print(passAP);
                        client.print("\"></dd><dd>Ka.: <input type=\"number\" min=\"1\" max=\"13\" id=\"kanalAP\" value=\"");
                        client.print(kanalAP);
                        client.println("\"></dd></dl>");
                        
                        client.print("<h2>WiFi client</h2>");
                        client.print("<dl><dd>IP: ");
                        if (WiFi.status() == WL_CONNECTED)
                          client.print(WiFi.localIP());
                        else client.print("none");
                        client.print("</dd><dd>SSID: <input type=text id=\"ssid\" value=\"");  
                        client.print(ssid);
                        client.print("\"></dd><dd>code: <input type=text id=\"pass\" value=\"");
                        client.print(pass);
                        client.println("\"></dd></dl>");

                        client.println("<h2>S88 Module</h2>");
                        client.print("<dl><dd>8x Anzahl: <input type=number min=\"0\" max=\"62\" id=\"S88\" value=\"");
                        #if defined(S88N)                        
                          client.print(S88Module);
                          client.print("\"");
                        #else
                          client.print("0\" disabled");
                        #endif  
                        client.println("></dd></dl><br>");
                        
                        client.println("<input type=submit onclick=\"SetState()\">"); 
                        client.println("<p id=\"state\"></p>");
                        client.print("<hr><p>Z21_ESP_Central_UDP_v");
                        client.print(Z21mobileVERSIONMSB);
                        client.print(".");
                        client.print(Z21mobileVERSIONLSB);
                        client.print(SwitchFormat);
                        #if defined (BOOSTER_INT_NDCC)
                        if (FIXSTORAGE.read(EEPROMRailCom) == 0x01)
                          Debug.print(".RAILCOM");
                        #endif
                        client.println("<br>Copyright (c) 2018 Philipp Gahtow<br>digitalmoba@arcor.de</p>");
                        client.println("</body>");
                        client.print("</html>");
                    }
                    // display received HTTP request on serial port
                    //Serial.print(HTTP_req);
                    HTTP_req = "";            // finished with request, empty string
                    break;
                }
                // every line of text received from the client ends with \r\n
                if (c == '\n') {
                    // last character on line of received text
                    // starting new line with next character read
                    currentLineIsBlank = true;
                } 
                else if (c != '\r') {
                    // a text character was received from client
                    currentLineIsBlank = false;
                }
            } // end if (client.available())
        } // end while (client.connected())
        delay(1);      // give the web browser time to receive the data
        client.stop(); // close the connection
    } // end if (client)
}
#endif

/*
//--------------------------------------------------------------
//DCC handle back updated loco information:
void notifyLokAll(uint16_t Adr, uint8_t Steps, uint8_t Speed, uint8_t F0, uint8_t F1, uint8_t F2, uint8_t F3)
{     
  #if defined(XPRESSNET)  
  if (XNetReturnLoco > 0) {
    if (Steps == 0x03)  //128 Steps?
        Steps++;  //set Steps to 0x04
    if ((XNetReturnLoco & 0x01) > 0) {
      bitWrite(XNetReturnLoco,0,0);
      XpressNet.SetLocoInfo(XNetUserOps, Steps, Speed, F0, F1); //Steps,Speed,F0,F1
    }
    if ((XNetReturnLoco & 0x02) > 0) {
      bitWrite(XNetReturnLoco,1,0);
      XpressNet.SetFktStatus(XNetUserOps, F2, F3); //Fkt4, Fkt5
    }
    if ((XNetReturnLoco & 0x04) > 0) {
      bitWrite(XNetReturnLoco,2,0);
      XpressNet.SetLocoInfoMM(XNetUserOps, Steps, Speed, F0, F1, F2, F3); //Steps,Speed,F0,F1,F2,F3
    }
    return;
  }
  else if (bc == false) {   //Wenn nicht durch XpressNet genutzt: setzte BUSY Flag
    if (XNetUserOps == 0) 
      XpressNet.ReqLocoBusy(Adr);   //Lok wird nicht von LokMaus gesteuert!
    //else XNetUserOps = 0;
  //}
  #endif
  
  #if defined(LOCONET) && defined(LnSLOTSRV)
//    LNSetLocoStatus(Adr, Speed, F0, F1);
  #endif

  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
//  z21.setLocoStateExt (Adr, Steps,Speed, F0, F1, F2, F3);
  
  #endif  
  
}
*/

//--------------------------------------------------------------
//DCC handle back the request switch state
void notifyTrnt(uint16_t Adr, bool State) 
{
  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
  z21.setTrntInfo(Adr, State);
  #endif
  
  #if defined(DEBUG)
  Debug.print(F("DCC Trnt "));
  Debug.print(Adr);
  Debug.print("-");
  Debug.println(State);
  #endif
}

//-------------------------------------------------------------- 
//DCC return a CV value:
void notifyCVVerify(uint16_t CV, uint8_t value) {
  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
  z21.setCVReturn (CV, value);
  #endif
  
  #if defined(XPRESSNET) 
  XpressNet.setCVReadValue(CV, value);
  #endif
  
  #if defined(DEBUG)
  Debug.print(F("Loco CV#"));
  Debug.print(CV);
  Debug.print(" - ");
  Debug.println(value);
  #endif
}

//-------------------------------------------------------------- 
//DCC return no ACK:
void notifyCVNack() {
  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
  z21.setCVNack();  //send back to device and stop programming!
  #endif
  
  #if defined(XPRESSNET) 
  XpressNet.setCVNack();
  #endif
  
  #if defined(DEBUG)
  Debug.println("CV# no ACK");
  #endif
}

//-------------------------------------------------------------- 
//DCC handle railpower while programming (Service Mode ON/OFF)
void notifyRailpower(uint8_t state) {
  globalPower(state); //send Power state to all Devices!
}

//--------------------------------------------------------------------------------------------
#if defined(DEBUG) && !defined(DUE_MCU) && !defined(ESP_MCU)
//ONLY for Atmega, not for Arduino DUE or ESP chip (ARM)!
int freeRam () 
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
#endif

//--------------------------------------------------------------
//INIT all ports and interfaces:
void setup() {
  /* Not nessesary: -> no support for all platforms!!
   * //Reduce the reference Voltage to 1,1 Volt:
  #if defined(BOOSTER_INT_MAINCURRENT)
    #if defined(MEGA_MCU) //Arduino MEGA
      analogReference(INTERNAL1V1); //or INTERNAL2V56
    #else  //others Arduino's
      analogReference(INTERNAL);  //1,1 Volt
    #endif   
    pinMode(VAmpIntPin, INPUT); 
  #endif  
  */
  pinMode(DCCLed, OUTPUT);      //DCC Status LED
  digitalWrite(DCCLed, LOW);    //DCC LED is in "off" State
  pinMode(ShortLed, OUTPUT);    //Short Status LED
  digitalWrite(ShortLed, HIGH);    //Short LED showes working and Power up
  pinMode(ProgRelaisPin, OUTPUT);       //ProgTrack-Relais
  digitalWrite(ProgRelaisPin, LOW);     //ProgTrack 
  #if defined(BOOSTER_EXT)    //Booster (ROCO) external: 
    pinMode(ShortExtPin, INPUT_PULLUP);  //set short pin and Turn on internal Pull-Up Resistor
    pinMode(GoExtPin, OUTPUT);      //GO/STOP Signal
    digitalWrite(GoExtPin, BOOSTER_EXT_OFF);    //set STOP to Booster
  #endif
  #if defined(BOOSTER_INT)    //Booster2 internal:
    #if !defined(BOOSTER_INT_NDCC)
    pinMode(GoIntPin, OUTPUT);    //GO/STOP2 Signal
    digitalWrite(GoIntPin, BOOSTER_INT_OFF);   //set STOP to Booster2 invertet
    #endif
    pinMode(ShortIntPin, INPUT_PULLUP);  //set up short2 PIN and Turn on internal Pull-Up Resistor
  #endif
  pinMode(Z21ResetPin, INPUT_PULLUP); //Turn on internal Pull-Up Resistor
  pinMode(Z21ButtonPin, INPUT_PULLUP); //Turn on internal Pull-Up Resistor

  #if defined(MEGA_MCU)
  pinMode(VAmSencePin, INPUT_PULLUP);  //AC 5A Sensor (for testing only)
  pinMode(VoltIntPin, INPUT_PULLUP);  //Rail Voltage: Rail:100k - Sence - 4,7k - GND
  pinMode(TempPin, INPUT_PULLUP);     //Temp.Resistor(15k)
  #endif

  #if defined(ESP_WIFI)
  EEPROM.begin(EESize);  //init EEPROM
  #endif

  #if defined(DEBUG) || defined(LnDEB) || defined(Z21DEBUG) || defined(REPORT) || defined(XnDEB)
    Debug.begin(DebugBaud);
    #if defined(ESP_MCU)
    Debug.println();  //Zeilenumbruch einfügen
    #endif
    Debug.print(F("Z21 "));
    Debug.print(Z21mobileVERSIONMSB);
    Debug.print(".");
    Debug.print(Z21mobileVERSIONLSB);
    Debug.print(SwitchFormat);
  #endif

  #if defined(DCC)
    //setup the DCC signal:
    #if defined(BOOSTER_INT_NDCC)
      #if defined(FS14)
      dcc.setup(DCCPin, GoIntPin, DCC14, SwitchFormat); 
      #elif defined(FS28)
      dcc.setup(DCCPin, GoIntPin, DCC28, SwitchFormat); 
      #else
      dcc.setup(DCCPin, GoIntPin, DCC128, SwitchFormat); 
      #endif
    #else
      #if defined(FS14)    
      dcc.setup(DCCPin, 0, DCC14, SwitchFormat);  //no NDCC and no RAILCOM
      #elif defined(FS28)    
      dcc.setup(DCCPin, 0, DCC28, SwitchFormat);  //no NDCC and no RAILCOM
      #else
      dcc.setup(DCCPin, 0, DCC128, SwitchFormat);  //no NDCC and no RAILCOM
      #endif
    #endif  
    //for CV reading activate the current control:
    #if defined(BOOSTER_INT_MAINCURRENT) 
      dcc.setCurrentLoadPin(VAmpIntPin);
    #endif
    //for CV reading over RAILCOM activate i2c communication:
    #if defined(DCCGLOBALDETECTOR)
      #if defined(RAILCOMI2C)
      Wire.begin(1);                // join i2c bus with address #8
      Wire.onReceive(RailComReceiveEvent); // register event
      #endif
      #if defined(MEGA_MCU)
      RailComSetup(); //init the Serial interface for receiving RailCom
      #endif
    #endif
    //extra DCC Output for S88 or LocoNet
    #if defined(additionalOutPin)
      #if (!defined(LAN) && defined (UNO_MCU)) || defined(MEGA_MCU) || defined(DUE_MCU)
        dcc.enable_additional_DCC_output(additionalOutPin);
        #if defined(DEBUG)
          Debug.print(".addOutput");
        #endif
      #endif
    #endif
  #endif

  #if defined(DEBUG) || defined(LnDEB) || defined(Z21DEBUG) || defined(REPORT) || defined(XnDEB)
    #if defined (BOOSTER_INT_NDCC)
      if (FIXSTORAGE.read(EEPROMRailCom) == 0x01)
        Debug.print(".RAILCOM");
    #endif
    #if defined(UNO_MCU)
    Debug.println(F(" - UNO"));
    #elif defined(MEGA_MCU)
    Debug.println(F(" - MEGA"));
    #elif defined(SANGUINO_MCU)
    Debug.println(F(" - SANGUINO"));
    #elif defined(DUE_MCU)
    Debug.println(F(" - DUE"));
    #elif defined(ESP_MCU)
    Debug.println(F(" - ESP"));
    #endif
    
  #endif

  #if defined(LAN)
  if ((digitalRead(Z21ButtonPin) == LOW) || (FIXSTORAGE.read(EEip) == 255)) {
      #if defined(DEBUG)
        Debug.println(F("RESET")); 
      #endif  
      FIXSTORAGE.FIXMODE(EEip, ip[0]);
      FIXSTORAGE.FIXMODE(EEip+1, ip[1]);
      FIXSTORAGE.FIXMODE(EEip+2, ip[2]);
      FIXSTORAGE.FIXMODE(EEip+3, ip[3]);
      while (digitalRead(Z21ButtonPin) == LOW) {  //Wait until Button - "UP"
        #if defined(DEBUG)
          Debug.print("."); 
        #endif  
        delay(200);   //Flash:
        digitalWrite(DCCLed, !digitalRead(DCCLed));
        digitalWrite(ShortLed, !digitalRead(DCCLed));
      }
      #if defined(DEBUG)
          Debug.println();  //new line!
      #endif  
      digitalWrite(DCCLed, LOW);    //DCC LED is in "off" State
      digitalWrite(ShortLed, LOW);    //Short LED is in "off" State
  }
  
  ip[0] = FIXSTORAGE.read(EEip);
  ip[1] = FIXSTORAGE.read(EEip+1);
  ip[2] = FIXSTORAGE.read(EEip+2);
  ip[3] = FIXSTORAGE.read(EEip+3);
  #endif

  #if defined(S88N)
    SetupS88();    //S88 Setup 
  #endif  

  #if defined(XPRESSNET)  
    #ifdef __SAM3X8E__
    XpressNet_DUE_setup();
    #endif
    #if defined(FS14)
    XpressNet.setup(Loco14, XNetTxRxPin);    //Initialisierung XNet Serial und Send/Receive-PIN  
    #elif defined(FS28)
    XpressNet.setup(Loco28, XNetTxRxPin);    //Initialisierung XNet Serial und Send/Receive-PIN  
    #else
    XpressNet.setup(Loco128, XNetTxRxPin);    //Initialisierung XNet Serial und Send/Receive-PIN  
    #endif
  #endif

  #if defined(DECODER)
    DCCDecoder_init();    //DCC Decoder init
  #endif
  
  #if defined(LOCONET)
    LNsetup();      //LocoNet Interface init
  #endif

  #if defined(WIFI)
    WLANSetup();    //Start ESP WLAN
  #endif 

  #if defined(ESP_WIFI)
    ESPSetup();   //ESP8266 Setup
    #if defined(ESP_HTTPCONF)
      server.begin(); //Start the HTTP server
    #endif
    Udp.begin(z21Port);   //open Z21 port
  #endif

  #if defined(LAN)
    // start the Ethernet and UDP:
    delay(100); //wait for ethernet to get up
    #if defined(DHCP)
      #if defined(DEBUG)
          Debug.print(F("IP over DHCP..."));  
      #endif  
      if (Ethernet.begin(mac) == 0) { //Trying to get an IP address using DHCP
        #if defined(DEBUG)
          Debug.println(F("fail!"));  
        #endif
        #undef DHCP
      }
      else {
        //Save IP that receive from DHCP
        ip = Ethernet.localIP();
        #if defined(DEBUG)
          Debug.println("OK");  
        #endif
      }
    #endif
    #if !defined(DHCP)
      // initialize the Ethernet device not using DHCP:
      Ethernet.begin(mac,ip);  //set IP and MAC  
    #endif
    Udp.begin(z21Port);  //UDP Z21 Port 21105

  //UdpMT.begin(34472);   //UDP Maintenance Tool
  //0x30 0x80 0x01 0x02

    #if defined(HTTPCONF)
      server.begin();    //HTTP Server
    #endif
  #endif

  #if defined(DEBUG)
    #if defined(LAN)
    Debug.print(F("Eth IP: "));
    Debug.println(ip);
    #endif
    #if defined(S88N)
      Debug.print(F("S88: "));
      Debug.println(S88Module);
    #endif
    #if !defined(DUE_MCU) && !defined(ESP_MCU) //not for the DUE or ESP!
      Debug.print(F("RAM: "));
      Debug.println(freeRam());  
    #endif
  #endif  

  #if defined(DALLASTEMPSENSE) && defined(MEGA_MCU)
  sensors.begin();
  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 9);
  #endif

  globalPower(Railpower); //send Power state to all Devices!

  digitalWrite(ShortLed, LOW);    //Short LED goes off - we are ready to work!

}

//--------------------------------------------------------------------------------------------
//run the state machine to update all interfaces
void loop() {

  updateLedButton();     //DCC Status LED and Button

  #if defined(DCC)
  ShortDetection();  //handel short on rail to => power off
  dcc.update();    //handel Rail Data
  
    #if defined(DCCGLOBALDETECTOR) && defined(MEGA_MCU)
    RailComRead();  //check RailCom Data
    #endif
    
  #endif

  #if (defined(HTTPCONF) && defined(LAN)) || (defined(ESP_MCU) && defined(ESP_HTTPCONF))
    Webconfig();    //Webserver for Configuration
  #endif
  
  #if defined(S88N)
    notifyS88Data();    //R-Bus geänderte Daten 1. Melden
  #endif  
  
  #if defined(DECODER)
    DCCDecoder_update();    //Daten vom DCC Decoder
  #endif
  
  #if defined(XPRESSNET)  
    #ifdef __SAM3X8E__
    XpressNet_DUE_update(); //call in every loop
    #endif
    XpressNet.update(); //XpressNet Update
  #endif

  #if defined(LOCONET)
    LNupdate();      //LocoNet update
  #endif
  
  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
    Z21LANreceive();   //Z21 LAN Decoding
  #endif

}
