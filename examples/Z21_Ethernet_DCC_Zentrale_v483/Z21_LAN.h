//Z21 LAN and WiFi 
//Kommunikation mit der Z21 Library!
//
//Copyright (c) by Philipp Gahtow, year 2018
/*
 * Add change für WLAN für multi Paket - meherer Abfragen vom Client
 * Add change via LAN für multi Paket
 * Add store for source IP
 */
//----------------------------------------------
#if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)

// buffers for receiving and sending data
#define Z21_UDP_TX_MAX_SIZE 20  //--> Z21 LocoNet tunnel DATA has 20 Byte!

//----------------------------------------------
#if defined(LAN) || defined(ESP_WIFI)
#define Z21_Eth_offset WLANmaxIP   //shift LAN client behind WiFi client
#endif

//----------------------------------------------
#if defined(ESP_WIFI)
//client ssid and pw stored in EEPROM!
String ssid = "";  
String pass = "";
//AP ssid and pw read out from EEPROM:
String ssidAP = "";
String passAP = "";
byte kanalAP = 3;
#endif

//----------------------------------------------
#if defined(LAN) || defined(ESP_WIFI)
typedef struct    //Rückmeldung des Status der Programmierung
{
  byte IP0; //client IP-Adresse
  byte IP1;
  byte IP2;
  byte IP3;
  byte time;  //aktive Zeit
  unsigned int port;    //source Port
} listofIP;
listofIP mem[LANmaxIP];
byte storedIP = 0;  //speicher für IPs
#endif

//----------------------------------------------
unsigned long IPpreviousMillis = 0;       // will store last time of IP decount updated and SystemInfo update

#if defined(WIFI)
byte outData[Z21_UDP_TX_MAX_SIZE];    //store received Serial to send via UDP
byte outDcount = 0;  //position writing out data
byte sendTOO = 0xFF;  //memIP to whom to send the data
#endif

//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
#if defined(WIFI)
void WLANSetup() {
    digitalWrite(DCCLed, HIGH);   //show Waiting for WiFi Modul
    WLAN.begin(WIFISerialBaud);  //UDP to Serial Kommunikation
    byte b0 = 0;
    byte b1 = 0;
    #if defined(DEBUG)
    Debug.print(WIFISerialBaud);
    Debug.print(F("b start WiFi.."));
    #endif
    byte timeout = 20;  //try to init at least
    do {
      #if defined(DEBUG)
      Debug.print(".");
      #endif
      WLAN.write(0xFE);
      WLAN.print(F("S88"));
      #if defined(S88N)
      WLAN.write(S88Module);  
      #else 
      WLAN.write(0xFE);
      #endif
      delay(800);   //wait for ESP8266 to receive!
      while (WLAN.available() > 0) {
         b0 = b1;
         b1 = WLAN.read();
         #if defined(DEBUG)
         Debug.write(b1);
         #endif
      }
      timeout--;
    }  
    while (b0 != 'O' && b1 != 'K' && timeout > 0);
    #if defined(DEBUG)
    if (timeout == 0)
      Debug.println("FAIL");
    else {
      Debug.println();
      #if defined(DEBUG_WLAN_CONFIG)  
      //Report ESP config data: ---------------------------------------------------
      String list[] = {F("WiFi-AP"),F("SSIP-AP"),F("PASS-AP"),F("CHAN-AP"),F("WiFi-CL"),F("SSIP-CL"),F("PASS-CL"),F("S88")};
      //request each singel value:
      for (byte counter = 0; counter < 8; counter++) {  
        WLAN.write(0xFE);
        WLAN.print(F("get"));
        WLAN.write(counter);
        timeout = 0xFF;
        while (WLAN.available() == 0 && timeout > 0) {timeout--; delay(5);}  //wait
        if (timeout > 0) {
          Debug.print(list[counter]);
          Debug.print(": ");
          while (WLAN.available() > 0) {
            Debug.write(WLAN.read());
          }
          Debug.println();
        }
        else return;  //no answer: break getting config data!
      }
      #endif
      //END ESP config data --------------------------------------------------------
    }
    #endif   
    digitalWrite(DCCLed, LOW);  //Wifi detecting finish!
}
#endif

//--------------------------------------------------------------------------------------------
#if defined(ESP_WIFI)
/**********************************************************************************/
void EEPROMwrite (String s, byte laenge, byte start) {
  byte len = s.length();
  EEPROM.write(laenge,len);
  for (int i = start; i < (start+len); i++) {
     EEPROM.write(i,s[i-start]);
  }
}
/**********************************************************************************/
String EEPROMread (byte laenge, byte start) {
  String s = "";
  byte len = EEPROM.read(laenge);
  for (int i = start; i < (start+len); i++) {
    s += char(EEPROM.read(i));
  }
  return s;
}
/**********************************************************************************/
boolean tryWifiClient() {
  // WiFi.scanNetworks will return the number of networks found
  byte n = 1; //WiFi.scanNetworks();
  if ((n > 0) && (ssid.length() > 0)) {
    //Try to connect to WiFi
     WiFi.begin(ssid.c_str(), pass.c_str());
     n = 0;
     while(WiFi.waitForConnectResult() != WL_CONNECTED){
//        WiFi.begin(ssid.c_str(), pass.c_str());
        delay(100);
        n++;
        if (n > 30) {
            WiFi.disconnect();
            return false;
        }
      }
      return true;
  }
  return false;
}
/**********************************************************************************/
void ESPSetup() {
// read eeprom for ssid and pass
  //--------------WIFI CLIENT---------------
  ssid = EEPROMread(EEssidLength, EEssidBegin);
  pass = EEPROMread(EEpassLength, EEpassBegin);
  //--------------ACCESS POINT------------- 
  ssidAP = EEPROMread(EEssidAPLength, EEssidAPBegin);
  passAP = EEPROMread(EEpassAPLength, EEpassAPBegin);
  if ((ssidAP.length() > 32) || (ssidAP.length() == 0) || (passAP.length() > 32) || (passAP.length() == 0) ) { //request is OK?
    ssidAP = SssidAP;
    passAP = SpassAP;
    kanalAP = SkanalAP;
          EEPROMwrite (ssidAP, EEssidAPLength, EEssidAPBegin);
          EEPROMwrite (passAP, EEpassAPLength, EEpassAPBegin);
          EEPROM.write(EEkanalAP, kanalAP);
          EEPROM.commit();
  }
  else {    
      kanalAP = EEPROM.read(EEkanalAP);
      if ((kanalAP < 1) || (kanalAP > 13))
        kanalAP = SkanalAP;
  }

  WiFi.mode(WIFI_AP_STA);  //AP & client
  
  //WiFi.softAPConfig(Ip, Gw, Sb);  //set the IP for Z21
  WiFi.softAP(ssidAP.c_str(), passAP.c_str(), kanalAP, false);    //Start AcessPoint
  //don't hide SSID and max simultaneous connected stations set to eight!
  
  tryWifiClient();  //Try to connect to WiFi  
}
#endif

//--------------------------------------------------------------------------------------------
#if defined(LAN) || defined(ESP_WIFI)
byte Z21addIP (byte ip0, byte ip1, byte ip2, byte ip3, unsigned int port) {
  //suche ob IP schon vorhanden?
  for (byte i = 0; i < storedIP; i++) {
    if (mem[i].IP0 == ip0 && mem[i].IP1 == ip1 && mem[i].IP2 == ip2 && mem[i].IP3 == ip3 && mem[i].port == port) {
      mem[i].time = ActTimeIP; //setzte Zeit
      return i+1;
    }
  }
  if (storedIP >= LANmaxIP) {
    for (byte i = 0; i < storedIP; i++) {
      if (mem[i].time == 0) { //Abgelaufende IP, dort eintragen!
         mem[i].IP0 = ip0;
         mem[i].IP1 = ip1;
         mem[i].IP2 = ip2;
         mem[i].IP3 = ip3;
         mem[i].port = port;
         mem[i].time = ActTimeIP; //setzte Zeit
         return i+1;
      }
    }
    return 0; //keine freien IPs (never reach here!)
  }
  mem[storedIP].IP0 = ip0;
  mem[storedIP].IP1 = ip1;
  mem[storedIP].IP2 = ip2;
  mem[storedIP].IP3 = ip3;
  mem[storedIP].port = port;
  mem[storedIP].time = ActTimeIP; //setzte Zeit
  storedIP++;
  return storedIP;  
}
#endif

//--------------------------------------------------------------------------------------------
void Z21LANreceive () {
  //-------------------------------------------------------------------------------------------
  //receive LAN data or we run on ESP MCU:
  #if defined(LAN) || defined(ESP_WIFI)
  byte packetSize = Udp.parsePacket();
  if(packetSize) {  //packetSize
    IPAddress remote = Udp.remoteIP();
    byte packetBuffer[packetSize];
    /*byte len =*/ Udp.read(packetBuffer,packetSize);  // read the packet into packetBufffer
    if (packetSize == packetBuffer[0]) { //normal:
      #if defined(Z21DATADEBUG)
      Debug.print(Z21addIP(remote[0], remote[1], remote[2], remote[3], Udp.remotePort()) + Z21_Eth_offset);
      Debug.print(" Z21 RX: ");
      for (byte i = 0; i < packetBuffer[0]; i++) {
        Debug.print(packetBuffer[i], HEX);
        Debug.print(" ");
      }
      Debug.println();
      #endif
      z21.receive(Z21addIP(remote[0], remote[1], remote[2], remote[3], Udp.remotePort()) + Z21_Eth_offset, packetBuffer);
    }
    else {  //kombiniertes UDP Paket:
      byte readpos = 0;   //position des aktuellen Paket
      //durchlaufe alle Pakete:
      do {    
        byte pBuffer[packetBuffer[readpos]];   //array of packet length
        for (byte i = 0; i < packetBuffer[readpos]; i++)    //fill up array with packet
          pBuffer[i] = packetBuffer[readpos + i];
        #if defined(Z21DATADEBUG)
        Debug.print(Z21addIP(remote[0], remote[1], remote[2], remote[3], Udp.remotePort()) + Z21_Eth_offset);
        Debug.print("-Z21 RX: ");
        for (byte i = 0; i < pBuffer[0]; i++) {
          Debug.print(pBuffer[i], HEX);
          Debug.print(" ");
        }
        Debug.println();
        #endif
        z21.receive(Z21addIP(remote[0], remote[1], remote[2], remote[3], Udp.remotePort()) + Z21_Eth_offset, pBuffer);
        readpos = packetBuffer[readpos] + readpos;  //bestimme position des nächsten Paket
      }
      while(readpos < packetSize);
    }
  }
  #endif

  //-------------------------------------------------------------------------------------------
  //receive WLAN data:
  #if defined(WIFI)
  while (WLAN.available() > 0) {  //Senden UDP
      outData[outDcount] = WLAN.read(); //read
      //Debug.write(outData[outDcount]);
      if (sendTOO == 0xFF)
        sendTOO = outData[outDcount];  //Ziel IP/client read out
      else {
        outDcount++;
        //kombinierte UDP Paket:
        if (outData[0] == 0) {  //Double Packet - check 2. Bit! 
          outData[0] = sendTOO;  //lese Länge des Packet
          sendTOO = outData[1]; //read back Stored client!
          outData[1] = 0;   //2. Bit der länge ist immer "0"
          outDcount++;  //goto next Bit
        }
        if (outData[0] <= outDcount) {  //1. Byte gibt länge der Daten an!
          if (sendTOO <= WLANmaxIP) {     //Ziel in range?
              //read Data:
            #if defined(Z21DATADEBUG)  
            Debug.print(sendTOO);
            Debug.print(" Z21 RX: ");
            for (byte i = 0; i < outData[0]; i++) {
              Debug.print(outData[i], HEX);
              Debug.print(" ");
            }
            Debug.println(F(" WLAN ok"));  //Accept
            #endif
            z21.receive(sendTOO, outData);
            outData[1] = sendTOO;  //Store Client that send the DATA
          }
          #if defined(Z21DATADEBUG)
          else {
            Debug.print(F("Z21 EE "));  //Fail
            Debug.println(sendTOO, HEX);
          }
          #endif
          outDcount = 0;
          sendTOO = 0xFF;
        }
        else if ((outDcount >= Z21_UDP_TX_MAX_SIZE) || (sendTOO == 'E' && outData[0] == 'E') || (outData[outDcount-2] == 'E' && outData[outDcount-1] == 'E')) { 
          #if defined(Z21DATADEBUG)
          Debug.write(sendTOO);  //Fail?
          for (byte i = 0; i < outDcount; i++)
              Debug.write(outData[i]);
          Debug.println(F(" FLUSH!"));
          #endif
          outDcount = 0;  //reset read buffer
          sendTOO = 0xFF;
          //WLAN.flush();   //clear
        }
        else if ((sendTOO == 'O' && outData[0] == 'K') || (outData[outDcount-2] == 'O' && outData[outDcount-1] == 'K')) {  //keine valied Data!
          #if defined(Z21DATADEBUG)
          Debug.println(F("Z21 OK"));
          #endif
          outDcount = 0;  //reset read buffer
          sendTOO = 0xFF;
        }
        //S88 Module config:
        else if (sendTOO == 0xFE && outData[0] == 'S' && outData[1] == '8' && outData[2] == '8' && outDcount > 3) {
          #if defined(S88N)
          if (outData[3] <= S88MAXMODULE) {  // && sendTOO >= 0
            S88Module = outData[3];
            #if defined(REPORT)
            Debug.print(F("Set S88 Module: "));
            Debug.println(S88Module);
            #endif
            if (FIXSTORAGE.read(EES88Moduls) != S88Module) {
              FIXSTORAGE.write(EES88Moduls, S88Module);
              SetupS88();
              WLANSetup();
            }
          }
          #else
          WLAN.write(0XFE);  
          WLAN.print(F("S88"));
          WLAN.write(0XFE);  //kein S88 aktiv!
          #endif
          outDcount = 0;   //reset read buffer
          sendTOO = 0xFF;
        }
      }
  }
  #endif
 
  //Nutzungszeit IP's bestimmen und update SystemInfo
  unsigned long currentMillis = millis();
  if((currentMillis - IPpreviousMillis) >= IPinterval) {
    IPpreviousMillis = currentMillis;   
    #if defined(LAN) || defined(ESP_WIFI)
    for (byte i = 0; i < storedIP; i++) {
        if (mem[i].time > 0) {
          mem[i].time--;    //Zeit herrunterrechnen
          #if defined(Z21DATADEBUG)
          if (mem[i].time == 0) {
            Debug.print(i);
            Debug.println(F(" LAN client off"));
          }
          #endif
        }
    }
    #endif
    #if defined(BOOSTER_INT_MAINCURRENT)
    notifyz21getSystemInfo(0); //SysInfo an alle BC Clients senden!
    #endif
  }
  
}

//--------------------------------------------------------------------------------------------
#if defined(S88N)
//Ask for Feedback data of group index:
void notifyz21S88Data(uint8_t gIndex) {
  //S88sendon = 'i';
  reportS88Data(gIndex);
}
#endif

//--------------------------------------------------------------------------------------------
void notifyz21RailPower(uint8_t State)
{
  #if defined(Z21DEBUG)  
    Debug.print(F("z21 PW: "));
  #endif          
  if (Railpower != State)
    globalPower(State);
}

//--------------------------------------------------------------------------------------------
void notifyz21LocoState(uint16_t Adr, uint8_t data[])
{
  #if defined(DCC) 
  dcc.getLocoData(Adr, data);
  #endif
}

//--------------------------------------------------------------------------------------------
void notifyz21LocoFkt(uint16_t Adr, uint8_t state, uint8_t fkt)
{
  #if defined(DCC) 
  dcc.setLocoFunc(Adr, state, fkt); 
  //dcc.getLocoStateFull(Adr);      //request for other devices
  #endif
  
  #if defined(LOCONET) && !defined(LnSLOTSRV)
  if (fkt <= 4) {
    byte DIRF = dcc.getFunktion0to4(Adr) | (dcc.getLocoDir(Adr) << 5);
    sendLNDIRF(Adr,  DIRF);
  }
  else if (fkt >= 5 && fkt <= 8)
    sendLNSND(Adr, dcc.getFunktion5to8(Adr));
  else if (fkt >= 9 && fkt <= 12)
    sendLNF3 (Adr, dcc.getFunktion9to12(Adr)); 
  else if (fkt >= 13 && fkt <= 20)
    sendLNF4 (Adr, dcc.getFunktion13to20(Adr));   
  else if (fkt >= 21 && fkt <= 28)
    sendLNF5 (Adr, dcc.getFunktion21to28(Adr));   
  #endif

  #if defined(XPRESSNET)
  if (fkt <= 4) 
    XpressNet.setFunc0to4(Adr, dcc.getFunktion0to4(Adr));
  else if (fkt >= 5 && fkt <= 8) 
    XpressNet.setFunc5to8(Adr, dcc.getFunktion5to8(Adr));
  else if (fkt >= 9 && fkt <= 12)
    XpressNet.setFunc9to12(Adr, dcc.getFunktion9to12(Adr));
  else if (fkt >= 13 && fkt <= 20)
    XpressNet.setFunc13to20(Adr, dcc.getFunktion13to20(Adr));
  else if (fkt >= 21 && fkt <= 28)
    XpressNet.setFunc21to28(Adr, dcc.getFunktion21to28(Adr));
  XpressNet.ReqLocoBusy(Adr);   //Lok wird nicht von LokMaus gesteuert!  
  #endif
  
  #if defined(Z21DEBUG)
  Debug.print("Z21 A:");
  Debug.print(Adr);
  if (fkt <= 4) {
    Debug.print(", F1:");
    Debug.println(dcc.getFunktion0to4(Adr), BIN);
  }  
  else if (fkt >= 5 && fkt <= 8) {
    Debug.print(", F2:");
    Debug.println(dcc.getFunktion5to8(Adr), BIN);
  }
  else if (fkt >= 9 && fkt <= 12) {
    Debug.print(", F3:");
    Debug.println(dcc.getFunktion9to12(Adr), BIN);
  }
  else if (fkt >= 13 && fkt <= 20) {
    Debug.print(", F4:");
    Debug.println(dcc.getFunktion13to20(Adr), BIN);
  }
  else if (fkt >= 21 && fkt <= 28) {
    Debug.print(", F5:");
    Debug.println(dcc.getFunktion21to28(Adr), BIN);
  }
  #endif
}

//--------------------------------------------------------------------------------------------
void notifyz21LocoSpeed(uint16_t Adr, uint8_t speed, uint8_t steps)
{
  #if defined(LOCONET) && !defined(LnSLOTSRV)
  sendLNSPD(Adr, speed);
  #endif
  #if defined(XPRESSNET)
  XpressNet.setSpeed(Adr, steps, speed);
  XpressNet.ReqLocoBusy(Adr);   //Lok wird nicht von LokMaus gesteuert!
  #endif
  switch (steps) {
    case 14: dcc.setSpeed14(Adr, speed); break;
    case 28: dcc.setSpeed28(Adr, speed); break;
    default: dcc.setSpeed128(Adr, speed); 
  }
  #if defined(Z21DEBUG)
  Debug.print("Z21 A:");
  Debug.print(Adr);
  Debug.print(", S");
  Debug.print(steps);
  Debug.print(":");
  Debug.println(speed, BIN);
  #endif

  //#if defined(DCC) 
  //dcc.getLocoStateFull(Adr);      //request for other devices
  //#endif
}

//--------------------------------------------------------------------------------------------
void notifyz21Accessory(uint16_t Adr, bool state, bool active)
{
  dcc.setBasicAccessoryPos(Adr, state, active);
  #if defined(LOCONET)
  LNsetTrnt(Adr, state, active);
  #endif
  #if defined(XPRESSNET)
  XpressNet.SetTrntPos(Adr, state, active);
  #endif
}

//--------------------------------------------------------------------------------------------
uint8_t notifyz21AccessoryInfo(uint16_t Adr)
//return state of the Address (left/right = true/false)
{
  #if defined(Z21DEBUG)
  Debug.print("Z21 GetAccInfo: ");
  Debug.print(Adr);
  Debug.print("-");
  Debug.println(dcc.getBasicAccessoryInfo(Adr));
  #endif
  return dcc.getBasicAccessoryInfo(Adr);
}

#if defined(LOCONET)
//--------------------------------------------------------------------------------------------
uint8_t notifyz21LNdispatch(uint16_t Adr)
//return the Slot that was dispatched, 0xFF at error!
{
  #if defined(Z21DEBUG)
  Debug.print("Z21 LNdispatch: ");
  Debug.println(Adr);
  #endif
  return LNdispatch(Adr);
}
//--------------------------------------------------------------------------------------------
void notifyz21LNSendPacket(uint8_t *data, uint8_t length)
{
  #if defined(Z21DEBUG)
    Debug.println(F("LOCONET_FROM_LAN")); 
  #endif
  
  LNSendPacket (data, length);  
}

//--------------------------------------------------------------------------------------------
void notifyz21LNdetector(uint8_t typ, uint16_t Adr) {
  #if defined(Z21DEBUG)
  Debug.print(F("LAN_LOCONET_DETECTOR ")); 
  Debug.print(typ);
  Debug.print("=");
  Debug.println(Adr);
  #endif
  
  //z21.setLNDetector(data[], length);
}

#endif  //LOCONET

//--------------------------------------------------------------------------------------------
void notifyz21CANdetector(uint8_t typ, uint16_t ID) {
  #if defined(Z21DEBUG)
  Debug.print(F("CAN_DETECTOR ")); 
  Debug.print(typ);
  Debug.print("=");
  Debug.println(ID);
  #endif
  //z21.setCANDetector(uint16_t NID, uint16_t Adr, uint8_t port, uint8_t typ, uint16_t v1, uint16_t v2);
}

//--------------------------------------------------------------------------------------------
//read out the current of the booster main - in testing!

void notifyz21getSystemInfo(uint8_t client)
{
#if defined(BOOSTER_INT_MAINCURRENT)  
  uint16_t inAm = analogRead(VAmpIntPin) * 10;
  #if defined(MEGA_MCU)
  #if defined(DALLASTEMPSENSE)
  sensors.requestTemperatures(); // Send the command to get temperatures
  float temp = sensors.getTempCByIndex(0);
  #else
  uint16_t temp = analogRead(TempPin);
  temp = (1024-temp) / (temp / 11.5);
  #endif
  uint16_t volt = ((float)(analogRead(VoltIntPin)-121) / 0.008);
  #else //other MCU:
  uint16_t temp = 0;
  uint16_t volt = 0;
  #endif
  
  #if defined(Z21SYSTEMDATADEBUG)
  Debug.print("mA: ");
  Debug.print(inAm);
  Debug.print(" (");
  #if defined(MEGA_MCU) 
  Debug.print(analogRead(VAmSencePin)-512);  //AC 5A Sensor (for testing only)
  #endif
  Debug.print("), V: ");
  Debug.print(volt);  //Rail Voltage: Rail:100k - Sence - 4,7k - GND
  Debug.print(", T: ");
  Debug.println(temp);
  #endif
  z21.sendSystemInfo(client, inAm, volt, temp);  //report System State to z21 client
  //(12-22V): 20V=0x4e20, 21V=0x5208, 22V=0x55F0
#else
  z21.sendSystemInfo(client, 0, 0, 0);  //report zero System State to z21 client
#endif  
}


//--------------------------------------------------------------------------------------------
void notifyz21CVREAD(uint8_t cvAdrMSB, uint8_t cvAdrLSB)
{
  unsigned int cvAdr = cvAdrMSB << 8 | cvAdrLSB;
   #if defined(Z21DEBUG)
      Debug.print("Z21 CV get: ");
      Debug.println(cvAdr);
    #endif

  #if defined(DCC)  
  dcc.opsReadDirectCV(cvAdr);  //read cv
  #endif
}

//--------------------------------------------------------------------------------------------
void notifyz21CVWRITE(uint8_t cvAdrMSB, uint8_t cvAdrLSB, uint8_t value)
{
  unsigned int cvAdr = cvAdrMSB << 8 | cvAdrLSB;
   #if defined(Z21DEBUG)
      Debug.print("Z21 CV set: ");
      Debug.print(cvAdr);
      Debug.print(" - ");
      Debug.println(value);
    #endif

   #if defined(DCC) 
   dcc.opsProgDirectCV(cvAdr,value);  //return value from DCC via 'notifyCVVerify'
   #endif
}

//--------------------------------------------------------------------------------------------
void notifyz21CVPOMWRITEBYTE(uint16_t Adr, uint16_t cvAdr, uint8_t value)
{
  #if defined(Z21DEBUG)
  Debug.print(F("Z21 POM Byte set: "));
  Debug.print(Adr);
  Debug.print(" set ");
  Debug.print(cvAdr);
  Debug.print(" - ");
  Debug.println(value);
  #endif
  
  #if defined(DCC)
  dcc.opsProgramCV(Adr, cvAdr, value);  //set decoder byte
  #endif
}

//--------------------------------------------------------------------------------------------
void notifyz21CVPOMWRITEBIT(uint16_t Adr, uint16_t cvAdr, uint8_t value)
{
  #if defined(Z21DEBUG)
  Debug.print(F("Z21 POM Bit set: "));
  Debug.print(Adr);
  Debug.print(" set ");
  Debug.print(cvAdr);
  Debug.print(" - ");
  Debug.println(value);
  #endif
  
  #if defined(DCC)
  dcc.opsPOMwriteBit(Adr, cvAdr, value);  //set decoder bit
  #endif
}

//--------------------------------------------------------------------------------------------
void notifyz21CVPOMREADBYTE (uint16_t Adr, uint16_t cvAdr)
{
  #if defined(DCCGLOBALDETECTOR) && defined(DCC)

  #if defined(RCDEB)
  Debug.print(F("Z21 POM get: "));
  Debug.print(Adr);
  Debug.print(" get ");
  Debug.print(cvAdr+1);
  #endif

  RailcomCVAdr = cvAdr;
    
  #if defined(MEGA_MCU)
  RailComStart(); //Start Reading Data!
  #endif
  
  #endif

  #if defined(DCC)
  dcc.opsPOMreadCV(Adr, cvAdr);  //get decoder value
  #endif
}

//--------------------------------------------------------------------------------------------
//Information to DCC Libray via EEPROM (50 to 75) over RailCom, ProgModus, etc.
void notifyz21UpdateConf() {
  #if defined(DEBUG)
  Debug.println("Z21 Conf: ");
  Debug.print("Railcom: "); //RailCom: 0=aus/off, 1=ein/on
  Debug.print(FIXSTORAGE.read(EEPROMRailCom), HEX);
  Debug.print(", PWR-B: "); //Power-Button: 0=Gleisspannung aus, 1=Nothalt  
  Debug.print(FIXSTORAGE.read(52), HEX);
  Debug.print(", Prog-R: "); //Auslese-Modus: 0=Nichts, 1=Bit, 2=Byte, 3=Beides
  Debug.print(FIXSTORAGE.read(53), HEX);
  Debug.print(", RstP(s): ");
  Debug.print(FIXSTORAGE.read(EEPROMRSTsRepeat));
  Debug.print(", RstP(f): ");
  Debug.print(FIXSTORAGE.read(EEPROMRSTcRepeat));
  Debug.print(", ProgP: ");
  Debug.print(FIXSTORAGE.read(EEPROMProgRepeat));
  Debug.print(", MainV: ");
  Debug.print(word(FIXSTORAGE.read(73),FIXSTORAGE.read(72)));
  Debug.print(", ProgV: ");
  Debug.println(word(FIXSTORAGE.read(75),FIXSTORAGE.read(74)));
  #endif

  #if defined(DCC)
  dcc.loadEEPROMconfig();
  #endif
}

//--------------------------------------------------------------------------------------------
void notifyz21EthSend(uint8_t client, uint8_t *data) 
{
  #if defined(Z21DATADEBUG)
  Debug.print(client);
  Debug.print(F(" Z21 TX: "));
  for (byte i = 0; i < data[0]; i++) {
    Debug.print(data[i], HEX);
    Debug.print(" ");
  }
  #endif
  if (client == 0) { //all stored 
    #if defined(LAN) || defined(ESP_WIFI)
    for (byte i = 0; i < storedIP; i++) {
      if (mem[i].time > 0) {  //noch aktiv?
        IPAddress ip(mem[i].IP0, mem[i].IP1, mem[i].IP2, mem[i].IP3);
        Udp.beginPacket(ip, mem[i].port);    //Broadcast
        Udp.write(data, data[0]);
        Udp.endPacket();
      }
    }
    #endif
    
    #if defined(WIFI)
    WLAN.write(client);
    WLAN.write(data, data[0]);
    #endif
  }
  else {
    #if defined(LAN) || defined(ESP_WIFI)
    if (client < Z21_Eth_offset) {  //Prüfe über Offset ob WiFi or LAN
    #endif  
      #if defined(WIFI)
      WLAN.write(client);
      WLAN.write(data, data[0]);
      #endif
    #if defined(LAN) || defined(ESP_WIFI)
    }
    else {
      byte cl = client - Z21_Eth_offset - 1;  //senden ohne Offset!
      IPAddress ip(mem[cl].IP0, mem[cl].IP1, mem[cl].IP2, mem[cl].IP3);
      Udp.beginPacket(ip, mem[cl].port);    //no Broadcast
      Udp.write(data, data[0]);
      Udp.endPacket();
    }
    #endif
  }
}

//---------------------------------
#endif

