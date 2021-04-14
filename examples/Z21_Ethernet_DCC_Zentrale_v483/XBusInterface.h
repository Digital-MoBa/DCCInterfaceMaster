//--------------------------------------------------------------
/*

  XpressNetMaster Interface for Arduino
  
Funktionsumfang:  
- Fahren per LokMaus2 und MultiMaus
- Schalten von DCC Weichen mit der MultiMaus (not tested 15.04.15)

  Copyright (c) by Philipp Gahtow, year 2015
*/
#if defined(XPRESSNET)

//**************************************************************
//byte XNetUserOps = 0x00;
//byte XNetReturnLoco = 0x00;

//--------------------------------------------------------------
//Change Power Status
void notifyXNetPower(uint8_t State) {
  #if defined(XnDEB)
  Debug.print("XNetPower: ");
  Debug.println(State, HEX);
  #endif
  if (Railpower != State)
    globalPower(State);
}

//--------------------------------------------------------------
void notifyXNetgiveLocoInfo(uint8_t UserOps, uint16_t Address) {
  //XNetReturnLoco |= 0x01;
  //XNetUserOps = UserOps;
  #if defined(DCC) 
  //dcc.getLocoStateFull(Address, false); //request for XpressNet only!
  uint8_t ldata[6];
  dcc.getLocoData(Address, ldata);  //uint8_t Steps[0], uint8_t Speed[1], uint8_t F0[2], uint8_t F1[3], uint8_t F2[4], uint8_t F3[5]
  if (ldata[0] == 0x03)  //128 Steps?
      ldata[0]++;  //set Steps to 0x04
  XpressNet.SetLocoInfo(UserOps, ldata[0], ldata[1], ldata[2], ldata[3]); //UserOps,Steps,Speed,F0,F1
  #endif
}

//--------------------------------------------------------------
void notifyXNetgiveLocoFunc(uint8_t UserOps, uint16_t Address) {
  //XNetReturnLoco |= 0x02;
  //XNetUserOps = UserOps;
  #if defined(DCC) 
  //dcc.getLocoStateFull(Address, false); //request for XpressNet only!
  XpressNet.SetFktStatus(UserOps, dcc.getFunktion13to20(Address), dcc.getFunktion21to28(Address)); //Fkt4, Fkt5
  #endif
}

//--------------------------------------------------------------
void notifyXNetgiveLocoMM(uint8_t UserOps, uint16_t Address) {
  //XNetReturnLoco |= 0x04;
  //XNetUserOps = UserOps;
  #if defined(DCC) 
  //dcc.getLocoStateFull(Address, false); //request for XpressNet only!
  uint8_t ldata[6];
  dcc.getLocoData(Address, ldata);  //uint8_t Steps[0], uint8_t Speed[1], uint8_t F0[2], uint8_t F1[3], uint8_t F2[4], uint8_t F3[5]
  if (ldata[0] == 0x03)  //128 Steps?
      ldata[0]++;  //set Steps to 0x04
  XpressNet.SetLocoInfoMM(UserOps, ldata[0], ldata[1], ldata[2], ldata[3], ldata[4], ldata[5]); //Steps,Speed,F0,F1,F2,F3
  #endif
}

//--------------------------------------------------------------
void notifyXNetLocoDrive14(uint16_t Address, uint8_t Speed) {
  #if defined(LOCONET) && !defined(LnSLOTSRV)
  sendLNSPD(Address, map(Speed, -14, 14, -128, 128)); 
  #endif
  
  #if defined(XnDEB)
  Debug.print("XNet A:");
  Debug.print(Address);
  Debug.print(", S14:");
  Debug.println(Speed, BIN);
  #endif

  #if defined(DCC) 
  if (Speed == 0) 
    dcc.setSpeed14(Address, (dcc.getLocoDir(Address) << 7) | (Speed & B01111111));
  else dcc.setSpeed14(Address, Speed);
  //dcc.getLocoStateFull(Address);      //request for other devices
  #endif
  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
  z21.setLocoStateExt (Address);
  #endif
}

//--------------------------------------------------------------
void notifyXNetLocoDrive28(uint16_t Address, uint8_t Speed) {
  #if defined(LOCONET) && !defined(LnSLOTSRV)
  sendLNSPD(Address, map(Speed, -28, 28, -128, 128)); 
  #endif
  
  #if defined(XnDEB)
  Debug.print("XNet A:");
  Debug.print(Address);
  Debug.print(", S28:");
  Debug.println(Speed, BIN);
  #endif

  #if defined(DCC) 
  if (Speed == 0)
    dcc.setSpeed28(Address, (dcc.getLocoDir(Address) << 7) | (Speed & B01111111));
  else dcc.setSpeed28(Address, Speed);
  //dcc.getLocoStateFull(Address);      //request for other devices
  #endif
  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
  z21.setLocoStateExt (Address);
  #endif
}

//--------------------------------------------------------------
void notifyXNetLocoDrive128(uint16_t Address, uint8_t Speed) {
  #if defined(LOCONET) && !defined(LnSLOTSRV)
  sendLNSPD(Address, Speed);
  #endif
  
  #if defined(XnDEB)
  Debug.print("XNet A:");
  Debug.print(Address);
  Debug.print(", S128:");
  Debug.println(Speed, BIN);
  #endif

  #if defined(DCC) 
  //if ((Speed & 0x7F) == 0) 
//    dcc.setSpeed128(Address, (dcc.getLocoDir(Address) << 7) | (Speed & B01111111));
  //else 
  dcc.setSpeed128(Address, Speed);
  //dcc.getLocoStateFull(Address);      //request for other devices
  #endif
  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
  z21.setLocoStateExt (Address);
  #endif
}

//--------------------------------------------------------------
void notifyXNetLocoFunc1(uint16_t Address, uint8_t Func1) {
  #if defined(XnDEB)
  Debug.print("XNet A:");
  Debug.print(Address);
  Debug.print(", F1:");
  Debug.println(Func1, BIN);
  #endif

  #if defined(DCC) 
  dcc.setFunctions0to4(Address, Func1);	//- F0 F4 F3 F2 F1
  //dcc.getLocoStateFull(Address);      //request for other devices
  #endif
  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
  z21.setLocoStateExt (Address);
  #endif

  #if defined(LOCONET) && !defined(LnSLOTSRV)
  byte DIRF = Func1 | (!dcc.getLocoDir(Address) << 5);  //invertierte Fahrtrichtung!
    //Beim einschalten einer Funktion wird die Fahrtrichtung geändert - deshalb hier invertiert!
  sendLNDIRF(Address, DIRF);
  #endif
}

//--------------------------------------------------------------
void notifyXNetLocoFunc2(uint16_t Address, uint8_t Func2) {
  #if defined(XnDEB)
  Debug.print("XNet A:");
  Debug.print(Address);
  Debug.print(", F2:");
  Debug.println(Func2, BIN);
  #endif

  #if defined(DCC) 
  dcc.setFunctions5to8(Address, Func2);	//- F8 F7 F6 F5
  //dcc.getLocoStateFull(Address);      //request for other devices
  #endif
  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
  z21.setLocoStateExt (Address);
  #endif

  #if defined(LOCONET) && !defined(LnSLOTSRV)
  sendLNSND(Address, Func2);
  #endif
}

//--------------------------------------------------------------
void notifyXNetLocoFunc3(uint16_t Address, uint8_t Func3) {
  #if defined(XnDEB)
  Debug.print("XNet A:");
  Debug.print(Address);
  Debug.print(", F3:");
  Debug.println(Func3, BIN);
  #endif

  #if defined(DCC) 
  dcc.setFunctions9to12(Address, Func3);	//- F12 F11 F10 F9
  //dcc.getLocoStateFull(Address);      //request for other devices
  #endif
  
  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
  z21.setLocoStateExt (Address);
  #endif

  #if defined(LOCONET) && !defined(LnSLOTSRV)
  sendLNF3(Address, Func3);
  #endif
}

//--------------------------------------------------------------
void notifyXNetLocoFunc4(uint16_t Address, uint8_t Func4) {
  #if defined(XnDEB)
  Debug.print("XNet A:");
  Debug.print(Address);
  Debug.print(", F4:");
  Debug.println(Func4, BIN);
  #endif

  #if defined(DCC) 
  dcc.setFunctions13to20(Address, Func4);	//F20 F19 F18 F17 F16 F15 F14 F13
  //dcc.getLocoStateFull(Address);      //request for other devices
  #endif

  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
  z21.setLocoStateExt (Address);
  #endif

  #if defined(LOCONET) && !defined(LnSLOTSRV)
  sendLNF4(Address, Func4);
  #endif
}

//--------------------------------------------------------------
void notifyXNetLocoFunc5(uint16_t Address, uint8_t Func5) {
  #if defined(XnDEB)
  Debug.print("XNet A:");
  Debug.print(Address);
  Debug.print(", F5:");
  Debug.println(Func5, BIN);
  #endif

  #if defined(DCC) 
  dcc.setFunctions21to28(Address, Func5);	//F28 F27 F26 F25 F24 F23 F22 F21
  //dcc.getLocoStateFull(Address);      //request for other devices
  #endif

  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
  z21.setLocoStateExt (Address);
  #endif

  #if defined(LOCONET) && !defined(LnSLOTSRV)
  sendLNF5(Address, Func5);
  #endif
}

//--------------------------------------------------------------
void notifyXNetTrntInfo(uint8_t UserOps, uint8_t Address, uint8_t data) {
  int adr = ((Address * 4) + ((data & 0x01) * 2));
  byte pos = data << 4;
  bitWrite(pos, 7, 1);  //command completed!
  if (dcc.getBasicAccessoryInfo(adr) == false)
    bitWrite(pos, 0, 1);
  else bitWrite(pos, 1, 1);  
  if (dcc.getBasicAccessoryInfo(adr+1) == false)
    bitWrite(pos, 2, 1);  
  else bitWrite(pos, 3, 1);    
  XpressNet.SetTrntStatus(UserOps, Address, pos);
  #if defined(XnDEB)
    Debug.print("XNet: ");
    Debug.print(adr);
    Debug.print(", P:");
    Debug.println(pos, BIN);
  #endif
}

//--------------------------------------------------------------
void notifyXNetTrnt(uint16_t Address, uint8_t data) {
    #if defined(XnDEB)
    Debug.print("XNet TA:");
    Debug.print(Address);
    Debug.print(", P:");
    Debug.println(data, BIN);
    #endif

    #if defined(DCC) 
    dcc.setBasicAccessoryPos(Address,data & 0x01, bitRead(data,3));    //Adr, left/right, activ
    #endif
    
    #if defined(LOCONET)
    LNsetTrnt(Address, data & 0x01, bitRead(data,3));   //send to LocoNet
    #endif
}

//--------------------------------------------------------------
void notifyXNetDirectCV(uint8_t CV, uint8_t data) {
  #if defined(DCC) 
  dcc.opsProgDirectCV(CV,data); 
  #endif
  #if defined(XnDEB)
  Debug.print("XNet CV:");
  Debug.print(CV);
  Debug.print(" - ");
  Debug.println(data);
  #endif
}

//--------------------------------------------------------------
void notifyXNetDirectReadCV(uint8_t cvAdr) {
  #if defined(DCC) 
  dcc.opsReadDirectCV(cvAdr);  //read cv
  #endif
  #if defined(XnDEB)
  Debug.print("XNet CV Read:");
  Debug.println(cvAdr);
  #endif
}

//--------------------------------------------------------------
void notifyXNetPOMwriteByte (uint16_t Adr, uint16_t CV, uint8_t data) {
  #if defined(XnDEB)
  Debug.print("XNet POM:");
  Debug.print(Adr);
  Debug.print(" CV");
  Debug.print(CV+1);
  Debug.print("-");
  Debug.println(data);
  #endif  
  #if defined(DCC)
  dcc.opsProgramCV(Adr, CV, data);  //set decoder byte
  #endif
}

//--------------------------------------------------------------
void notifyXNetPOMwriteBit (uint16_t Adr, uint16_t CV, uint8_t data) {
  #if defined(XnDEB)
  Debug.print("XNet POM Bit:");
  Debug.print(Adr);
  Debug.print("- CV: ");
  Debug.print(CV);
  Debug.print(" - ");
  Debug.println(data, BIN);
  #endif  
  #if defined(DCC)
  dcc.opsPOMwriteBit(Adr, CV, data);  //set decoder bit
  #endif
}

#endif
