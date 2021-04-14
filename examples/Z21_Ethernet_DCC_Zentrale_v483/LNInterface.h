//--------------------------------------------------------------
/*

  LocoNet Master Interface
  
Funktionsumfang:  
- Fahren aber nicht per Slot Write
- Dispatch (put via Z21)

Copyright (c) by Philipp Gahtow, year 2018
  
*/
#if defined(LOCONET)

//**************************************************************
//STAT1 => D2 = SL_SPDEX, D1 = SL_SPD14, D0 = SL_SPD28
#define LNLOCO14  0x02 //14 Speed Steps
#define LNLOCO28  0x01 //28 Speed Steps
#define LNLOCO128 0x07 //128 Speed Steps

//Setup up PIN-Configuration for different MCU
#include "MCU_config.h"

#if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
#include "z21header.h"
#endif

static void LNupdate();

//config:
#if defined(UNO_MCU) || defined(__AVR_ATmega644P__)
  #if !defined(WIFI) && !defined(LAN)
  #define MaxSlot 20
  #else
  #define MaxSlot 10
  #endif
#else
#define MaxSlot 120    //max. 120 Slots
#endif

typedef struct	//SLOT
{
  uint16_t LAdr;
  uint8_t Status;  
  /* Slot Status 1
      D7-SL_SPURGE
        1=SLOT purge en,ALSO adrSEL (INTERNAL use only, not seen on NET!)
        CONDN/CONUP: bit encoding-Control double linked Consist List

    2 BITS for Consist
      D6-SL_CONUP
      D3-SL_CONDN
        11=LOGICAL MID CONSIST , Linked up AND down
        10=LOGICAL CONSIST TOP, Only linked downwards
        01=LOGICAL CONSIST SUB-MEMBER, Only linked upwards
        00=FREE locomotive, no CONSIST indirection/linking

    2 BITS for BUSY/ACTIVE
      D5-SL_BUSY
      D4-SL_ACTIVE
        11=IN_USE loco adr in SLOT -REFRESHED
        10=IDLE loco adr in SLOT, not refreshed
        01=COMMON loco adr IN SLOT, refreshed
        00=FREE SLOT, no valid DATA, not refreshed

    3 BITS for Decoder TYPE encoding for this SLOT
      D2-SL_SPDEX
      D1-SL_SPD14
      D0-SL_SPD28
        010=14 step MODE
        001=28 step. Generate Trinary packets for this Mobile ADR
        000=28 step/ 3 BYTE PKT regular mode
        011=128 speed mode packets
        111=128 Step decoder, Allow Advanced DCC consisting
        100=28 Step decoder ,Allow Advanced DCC consisting
   */
} TypeSlot;

TypeSlot slot[MaxSlot];

lnMsg        *LnPacket;
//LnBuf        LnTxBuffer;

byte dispatchSlot = 0;    //To put and store a SLOT for DISPATCHING
boolean dpgetSLot = false;
//byte LNSendTX = 0;    //Z21 hat eine Meldung auf den LocoNet-Bus geschrieben?
boolean LNgetNext = false; //Z21 hat eine Meldung auf den LocoNet-Bus geschrieben?
boolean LNNextTX = false;
//--------------------------------------------------------------------------------------------
//Define new OPC:
#ifndef OPC_UHLI_FUN
#define OPC_UHLI_FUN   0xD4  //Function 9-28 by Uhlenbrock 
#endif
#ifndef OPC_MULTI_SENSE
#define OPC_MULTI_SENSE 0xD0  //power management and transponding
#endif

#if defined(LnDEB)
//Print LN Message
void LNDebugPrint () {
  uint8_t msgLen = getLnMsgSize(LnPacket);   //get Length of Data
  // First print out the packet in HEX
        for (uint8_t x = 0; x < msgLen; x++)  {
          uint8_t val = LnPacket->data[x];
            // Print a leading 0 if less than 16 to make 2 HEX digits
          if(val < 16)
            Debug.print('0');
          Debug.print(val, HEX);
          Debug.print(' ');
        }
        Debug.println();
}
#endif

//--------------------------------------------------------------------------------------------
//send bytes into LocoNet
bool LNSendPacket (byte *data, byte len) 
{
  lnMsg SendPacket;
  for (byte i = 0; i < (len-1); i++) {
    SendPacket.data[i] = *data;
    data++;
  }
  LocoNet.send( &SendPacket);
 
  LnPacket = LocoNet.receive();
  if (SendPacket.data[len-1] == LnPacket->data[len-1] && SendPacket.data[0] == LnPacket->data[0] && SendPacket.data[1] == LnPacket->data[1]) {
    //LNNextTX = true;
    #if defined(LnDEB)
    Debug.print("STX: ");
    LNDebugPrint();
    #endif
  }
  else {
    LNgetNext = true;
    LNupdate();
  }
  return true;
/*  
  static lnMsg   *LoconetPacket;
  byte XOR = 0xFF;
  for (byte i = 0; i < (length-1); i++) {
    addByteLnBuf( &LnTxBuffer, *data);
    XOR = XOR ^ *data;
    data++;
  }
  addByteLnBuf( &LnTxBuffer, XOR ) ;    //Trennbit
  addByteLnBuf( &LnTxBuffer, 0xFF ) ;    //Trennbit
  // Check to see if we have received a complete packet yet
  LoconetPacket = recvLnMsg( &LnTxBuffer );    //Senden vorbereiten
  if(LoconetPacket ) {        //korrektheit Prüfen
    LocoNet.send( LoconetPacket );  // Send the received packet from the PC to the LocoNet
    LNSendTX++;  //Meldung auf LocoNet geschreiben (TX) -> notify z21 LAN also
    
    LnPacket = LocoNet.receive() ;
    #if defined(LnDEB)
    Debug.print("STX: ");
    LNDebugPrint();
    #endif
    
    return true;
  }  
  return false;
  */
}

//--------------------------------------------------------------------------------------------
//Status req
void LNGetLocoStatus(byte Slot) {
  uint8_t ldata[6];
  dcc.getLocoData(slot[Slot].LAdr, ldata);  //uint8_t Steps[0], uint8_t Speed[1], uint8_t F0[2], uint8_t F1[3], uint8_t F2[4], uint8_t F3[5]
  
  if (Slot != 0) {    //wenn Lok im SLOT Server vorhanden ist --> update
    // DIRF = 0,0,DIR,F0,F4,F3,F2,F1 
    // SND = 0,0,0,0,F8,F7,F6,F5 
    byte DIRF = ((ldata[1] >> 2) & B00100000) | ldata[2];
    byte SLOT_DATA_READ[] = {OPC_SL_RD_DATA, 0x0E, Slot, slot[Slot].Status, lowByte(slot[Slot].LAdr & 0x7F), lowByte(ldata[1] & 0x7F), DIRF, 0, 0, highByte(slot[Slot].LAdr & 0x7F), lowByte(ldata[3] & 0x0F), 0, 0};
    LNSendPacket (SLOT_DATA_READ, 0x0E);
    #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
    z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, true);   
    #endif
  }
}

#if defined(LnSLOTSRV)
//--------------------------------------------------------------------------------------------
byte LNGetSetLocoSlot(unsigned int Adr, bool add) {
  if (Adr == 0)
    return 0;
  byte getSlot = 0;
  for (byte i = 1; i < MaxSlot; i++) {
    if (slot[i].LAdr == Adr)
      return i;  //already inside a SLOT
    if ((getSlot == 0) && (((slot[i].Status & 0x30) >> 4) == 0))
      getSlot = i;  //find a empty SLOT  
  }
  if (getSlot != 0 && add == true) {    //add Adr to SLOT Server!
    slot[getSlot].Status = 0x10;   //ACTIVE - COMMON loco adr IN SLOT
    #if defined(FS14)
    slot[getSlot].Status |= LNLOCO14;
    #elif defined(FS28)
    slot[getSlot].Status |= LNLOCO28;
    #else
    slot[getSlot].Status |= LNLOCO128; 
    #endif
    slot[getSlot].LAdr = Adr;
    return getSlot;
  }
  return 0;
}
#else //LN Slave-Mode
//--------------------------------------------------------------------------------------------
//Find Slot via Address
byte LNGetSetLocoSlot (unsigned int Adr, bool add) { //add only for MASTER-Mode!
  if (Adr == 0)
    return 0;
  for (byte i = 1; i < MaxSlot; i++) {
    if (slot[i].LAdr == Adr) { // && (getLNSlotState(Slot) != 0)) {
      return i;    //Vorhanden!
    }
  }
  //If not: Master puts Address into a SLOT
  byte getSlot[] = {OPC_LOCO_ADR, (Adr >> 7), (Adr & 0x7F), 0x00};  //Request loco address
  LNSendPacket(getSlot, 4);  
  return 0;
}
//--------------------------------------------------------------------------------------------
//Set slot direction, function 0-4 state 
// DIRF = 0,0,DIR,F0,F4,F3,F2,F1 
void sendLNDIRF (unsigned int Adr, byte DIRF) {
  byte Slot = LNGetSetLocoSlot(Adr, TXAllLokInfoOnLN);
  if (Slot > 0) {
    byte setDIRF[] = {OPC_LOCO_DIRF, Slot, DIRF, 0x00};
    LNSendPacket(setDIRF, 4);  
  }
}
//--------------------------------------------------------------------------------------------
//Set slot speed and update dir
void sendLNSPD (unsigned int Adr, byte SPD) {
  byte Slot = LNGetSetLocoSlot(Adr, TXAllLokInfoOnLN);
  if (Slot == 0)
    return;
  if (dcc.getLocoDir(Adr) != (SPD >> 7)) {
      byte DIRF = dcc.getFunktion0to4(Adr) | (dcc.getLocoDir(Adr) << 5);
      byte setDIRF[] = {OPC_LOCO_DIRF, Slot, DIRF, 0x00};
      LNSendPacket(setDIRF, 4);
      LNSendPacket(setDIRF, 4); //send twice because otherwise IB sometimes don't realize the change!
  }
  byte setSPD[] = {OPC_LOCO_SPD, Slot, 0x7F, 0x00};
  setSPD[2] &= SPD;
  LNSendPacket(setSPD, 4);  
}
//--------------------------------------------------------------------------------------------
//Set slot second function
// SND = 0,0,0,0,F8,F7,F6,F5 
void sendLNSND (unsigned int Adr, byte SND) {
  byte Slot = LNGetSetLocoSlot(Adr, TXAllLokInfoOnLN);
  if (Slot > 0) {
    byte setSND[] = {OPC_LOCO_SND, Slot, SND, 0x00};
    LNSendPacket(setSND, 4);  
  }
}

//--------------------------------------------------------------------------------------------
// F3 = 0,0,0,0,F12,F11,F10,F9 
void sendLNF3 (unsigned int Adr, byte F3) {
  byte Slot = LNGetSetLocoSlot(Adr, TXAllLokInfoOnLN);
  if (Slot > 0) {
    byte setF3[] = {0xA3, Slot, F3, 0x00};
    LNSendPacket(setF3, 4);  
  }
}

//--------------------------------------------------------------------------------------------
// F4 = F20,F19,F18,F17,F16,F15,F14,F13 
void sendLNF4 (unsigned int Adr, byte F4) {
  byte Slot = LNGetSetLocoSlot(Adr, TXAllLokInfoOnLN);
  if (Slot > 0) {
    byte setF4[] = {OPC_UHLI_FUN, 0x20, Slot, 0x08, 0x7F, 0x00}; // - F19-F13
    setF4[4] &= F4;
    LNSendPacket(setF4, 6);  
    byte F2028 = ((dcc.getFunktion21to28(Adr) & 0x80) >> 1) | ((F4 & 0x80) >> 2);  // - f28, f20, ---
    byte setF2028[] = {OPC_UHLI_FUN, 0x20, Slot, 0x05, F2028, 0x00};  // - F28,F20 ----
    LNSendPacket(setF2028, 6); 
  }
}
//--------------------------------------------------------------------------------------------
// F5 = F28,F27,F26,F25,F24,F23,F22,F21 
void sendLNF5 (unsigned int Adr, byte F5) {
  byte Slot = LNGetSetLocoSlot(Adr, TXAllLokInfoOnLN);
  if (Slot > 0) {
    byte setF5[] = {OPC_UHLI_FUN, 0x20, Slot, 0x09, 0x7F, 0x00}; // - F27-F21
    setF5[4] &= F5;
    LNSendPacket(setF5, 6);  
    byte F2028 = ((F5 & 0x80) >> 1) | ((dcc.getFunktion13to20(Adr) & 0x80) >> 2);  // - f28, f20, ---
    byte setF2028[] = {OPC_UHLI_FUN, 0x20, Slot, 0x05, F2028, 0x00};  // - F28,F20 ----
    LNSendPacket(setF2028, 6); 
  }
}
//--------------------------------------------------------------------------------------------
//Uhlenbrock F9 to F12 Message: A3 Slot F9-12 HEX (A3 09 01 54; A3 09 00 55)
//Uhlenbroch F13 to F20 and F21 to F28: D4 20 Slot F13-20 F20-21 HEX (D4 20 09 08 01 0B; D4 20 09 08 02 08)
/*
 * 18:39:03.972: [Rx - A3 03 01 5E]  Set (Intellibox-II format) loco in slot 3
F9=On F10=Off F11=Off F12=Off.
18:39:06.109: [Rx - A3 03 00 5F]  Set (Intellibox-II format) loco in slot 3
F9=Off F10=Off F11=Off F12=Off.
18:39:07.061: [Rx - A3 03 02 5D]  Set (Intellibox-II format) loco in slot 3
F9=Off F10=On F11=Off F12=Off.
18:39:07.934: [Rx - A3 03 00 5F]  Set (Intellibox-II format) loco in slot 3
F9=Off F10=Off F11=Off F12=Off.
18:39:10.290: [Rx - A3 03 04 5B]  Set (Intellibox-II format) loco in slot 3
F9=Off F10=Off F11=On F12=Off.
18:39:11.023: [Rx - A3 03 00 5F]  Set (Intellibox-II format) loco in slot 3
F9=Off F10=Off F11=Off F12=Off.
18:39:13.519: [Rx - A3 03 08 57]  Set (Intellibox-II format) loco in slot 3
F9=Off F10=Off F11=Off F12=On.
18:39:14.205: [Rx - A3 03 00 5F]  Set (Intellibox-II format) loco in slot 3
F9=Off F10=Off F11=Off F12=Off.

18:39:15.188: [Rx - D4 20 03 08 01 01]  Set (Intellibox-II format) loco in
slot 3 F13=On F14=Off F15=Off F16=Off F17=Off F18=Off F19=Off
18:39:16.483: [Rx - D4 20 03 08 00 00]  Set (Intellibox-II format) loco in
slot 3 F13=Off F14=Off F15=Off F16=Off F17=Off F18=Off F19=Off
18:39:23.441: [Rx - D4 20 03 08 02 02]  Set (Intellibox-II format) loco in
slot 3 F13=Off F14=On F15=Off F16=Off F17=Off F18=Off F19=Off
18:39:26.576: [Rx - D4 20 03 08 00 00]  Set (Intellibox-II format) loco in
slot 3 F13=Off F14=Off F15=Off F16=Off F17=Off F18=Off F19=Off
18:39:28.823: [Rx - D4 20 03 08 04 04]  Set (Intellibox-II format) loco in
slot 3 F13=Off F14=Off F15=On F16=Off F17=Off F18=Off F19=Off
18:39:30.227: [Rx - D4 20 03 08 00 00]  Set (Intellibox-II format) loco in
slot 3 F13=Off F14=Off F15=Off F16=Off F17=Off F18=Off F19=Off
18:39:42.176: [Rx - D4 20 03 08 08 08]  Set (Intellibox-II format) loco in
slot 3 F13=Off F14=Off F15=Off F16=On F17=Off F18=Off F19=Off
18:39:45.187: [Rx - D4 20 03 08 00 00]  Set (Intellibox-II format) loco in
slot 3 F13=Off F14=Off F15=Off F16=Off F17=Off F18=Off F19=Off
18:39:47.121: [Rx - D4 20 03 08 10 10]  Set (Intellibox-II format) loco in
slot 3 F13=Off F14=Off F15=Off F16=Off F17=On F18=Off F19=Off
18:39:48.291: [Rx - D4 20 03 08 00 00]  Set (Intellibox-II format) loco in
slot 3 F13=Off F14=Off F15=Off F16=Off F17=Off F18=Off F19=Off
18:40:02.300: [Rx - D4 20 03 08 20 20]  Set (Intellibox-II format) loco in
slot 3 F13=Off F14=Off F15=Off F16=Off F17=Off F18=On F19=Off
18:40:03.486: [Rx - D4 20 03 08 00 00]  Set (Intellibox-II format) loco in
slot 3 F13=Off F14=Off F15=Off F16=Off F17=Off F18=Off F19=Off
18:40:04.516: [Rx - D4 20 03 08 40 40]  Set (Intellibox-II format) loco in
slot 3 F13=Off F14=Off F15=Off F16=Off F17=Off F18=Off F19=On
18:40:05.296: [Rx - D4 20 03 08 00 00]  Set (Intellibox-II format) loco in
slot 3 F13=Off F14=Off F15=Off F16=Off F17=Off F18=Off F19=Off

18:40:09.710: [Rx - D4 20 03 05 20 2D]  Set (Intellibox-II format) loco in
slot 3 F20=On F28=Off
18:40:16.091: [Rx - D4 20 03 05 00 0D]  Set (Intellibox-II format) loco in
slot 3 F20=Off F28=Off

18:40:16.964: [Rx - D4 20 03 09 01 00]  Set (Intellibox-II format) loco in
slot 3 F21=On F22=Off F23=Off F24=Off F25=Off F26=Off F27=Off
18:40:26.449: [Rx - D4 20 03 09 00 01]  Set (Intellibox-II format) loco in
slot 3 F21=Off F22=Off F23=Off F24=Off F25=Off F26=Off F27=Off
18:40:29.554: [Rx - D4 20 03 09 02 03]  Set (Intellibox-II format) loco in
slot 3 F21=Off F22=On F23=Off F24=Off F25=Off F26=Off F27=Off
18:40:31.020: [Rx - D4 20 03 09 00 01]  Set (Intellibox-II format) loco in
slot 3 F21=Off F22=Off F23=Off F24=Off F25=Off F26=Off F27=Off
18:40:32.252: [Rx - D4 20 03 09 04 05]  Set (Intellibox-II format) loco in
slot 3 F21=Off F22=Off F23=On F24=Off F25=Off F26=Off F27=Off
18:40:33.329: [Rx - D4 20 03 09 00 01]  Set (Intellibox-II format) loco in
slot 3 F21=Off F22=Off F23=Off F24=Off F25=Off F26=Off F27=Off
18:40:46.885: [Rx - D4 20 03 09 08 09]  Set (Intellibox-II format) loco in
slot 3 F21=Off F22=Off F23=Off F24=On F25=Off F26=Off F27=Off
18:40:47.790: [Rx - D4 20 03 09 00 01]  Set (Intellibox-II format) loco in
slot 3 F21=Off F22=Off F23=Off F24=Off F25=Off F26=Off F27=Off
18:40:50.473: [Rx - D4 20 03 09 10 11]  Set (Intellibox-II format) loco in
slot 3 F21=Off F22=Off F23=Off F24=Off F25=On F26=Off F27=Off
18:40:51.128: [Rx - D4 20 03 09 00 01]  Set (Intellibox-II format) loco in
slot 3 F21=Off F22=Off F23=Off F24=Off F25=Off F26=Off F27=Off
18:40:56.183: [Rx - D4 20 03 09 20 21]  Set (Intellibox-II format) loco in
slot 3 F21=Off F22=Off F23=Off F24=Off F25=Off F26=On F27=Off
18:41:00.083: [Rx - D4 20 03 09 00 01]  Set (Intellibox-II format) loco in
slot 3 F21=Off F22=Off F23=Off F24=Off F25=Off F26=Off F27=Off
18:41:02.142: [Rx - D4 20 03 09 40 41]  Set (Intellibox-II format) loco in
slot 3 F21=Off F22=Off F23=Off F24=Off F25=Off F26=Off F27=On
18:41:03.484: [Rx - D4 20 03 09 00 01]  Set (Intellibox-II format) loco in
slot 3 F21=Off F22=Off F23=Off F24=Off F25=Off F26=Off F27=Off

18:41:06.292: [Rx - D4 20 03 05 40 4D]  Set (Intellibox-II format) loco in
slot 3 F20=Off F28=On
18:41:07.274: [Rx - D4 20 03 05 00 0D]  Set (Intellibox-II format) loco in
slot 3 F20=Off F28=Off
 */
//--------------------------------------------------------------------------------------------
#endif  //LN Slave Mode

//--------------------------------------------------------------------------------------------
//Check if Slot can be dispatched
byte LNdispatch (uint16_t Adr) {
  dispatchSlot = LNGetSetLocoSlot(Adr, true);  //add to SLOT
  #if !defined(LnSLOTSRV)  //At Slave Mode ask Master to dispatch 
   if (dispatchSlot > 0) {
    byte SetStat1[] = {OPC_SLOT_STAT1, dispatchSlot, 0x20, 0x00};
    LNSendPacket(SetStat1, 4); 
    byte NullMove[] = {OPC_MOVE_SLOTS, dispatchSlot, 0x00, 0x00};
    LNSendPacket(NullMove, 4); 
   }
   else { //no Slot for loco that should be dispatched - get a Slot!
      dpgetSLot = true;  //get ready for Slot
      return 0;
   }
  #endif
  if (((slot[dispatchSlot].Status & 0x30) >> 4) != B11) {  //not 11=IN_USE
    return dispatchSlot;
  }
  dispatchSlot = 0;  //clear
  return 0;
}

//--------------------------------------------------------------------------------------------
//LocoNet Interface init
void LNsetup() {
  // First initialize the LocoNet interface
  LocoNet.init(LNTxPin);
  //initLnBuf(&LnTxBuffer);
}

//--------------------------------------------------------------------------------------------
//Send railpower state to LocoNet devices
void LNsetpower() {
  byte code[] = { OPC_GPOFF, 0x00};
  if (Railpower == csNormal)
    code[0] = OPC_GPON;
  else if (Railpower == csEmergencyStop)
    code[0] = OPC_IDLE;  //B'cast emerg. STOP
  LNSendPacket(code,2);
  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, true);   
  #endif
}

//--------------------------------------------------------------------------------------------
//Trnt Daten senden
void LNsetTrnt(uint16_t Adr, boolean state, boolean active) {
  //OPC_SW_REQ
  //dcc.setBasicAccessoryPos(LnPacket->data[1] | ((LnPacket->data[2] & 0x0F) << 7),(LnPacket->data[2] >> 5) & 0x01, (LnPacket->data[2] >> 4) & 0x01);  //Adr, State, on/off
  byte lAdr = Adr & 0x7F;
  byte hAdr = (Adr >> 7) & 0x0F;
  byte Trnt[] = {OPC_SW_REQ, lAdr, hAdr, 0x00};
  bitWrite(Trnt[2], 5, state);
  bitWrite(Trnt[2], 4, active);
  LNSendPacket (Trnt, 4);  
  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, true);   
  #endif
}

//--------------------------------------------------------------------------------------------
//LocoNet update via each loop
void LNupdate() {
  // Check for any received LocoNet packets
  if (LNgetNext == false)
    LnPacket = LocoNet.receive() ;
  if( LnPacket ) {
    LNgetNext = false;
    #if defined(LnDEB)
      if (LNNextTX == true)
        Debug.print("LnTX: ");
      else Debug.print("LnRX: ");  
      LNDebugPrint();
    #endif
    if (LNNextTX == true) {
      LNNextTX = false;
      return;
    }
    //Broadcast-Flag:
    #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
    byte LnZ21bcType = Z21bcLocoNet_s;  //Z21bcLocoNet or Z21bcLocoNetLocos or Z21bcLocoNetSwitches
    #endif
    switch (LnPacket->data[0]) {  //OPC = Operation-Code
        case OPC_SL_RD_DATA: {  //for LN Slave-Mode ONLY
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, false);   
                  #endif
                  #if !defined(LnSLOTSRV)  //At Slave Mode ask Master to dispatch 
                   byte Slot = LnPacket->data[2];  //Slot#
                   slot[Slot].Status = LnPacket->data[3];
                   slot[Slot].LAdr = (LnPacket->data[9] << 7) | (LnPacket->data[4] & 0x7F); // -> Adr
                   if (dpgetSLot == true) {  //dispatch this Slot!
                    #if defined(LnDEB)
                    Debug.println("Dispatch");
                    #endif
                    if (((slot[Slot].Status & 0x30) >> 4) != 0x01) {    //not BUSY!
                      byte SetStat1[] = {OPC_SLOT_STAT1, Slot, (slot[Slot].Status & 0xC0) | (slot[Slot].Status & 0x0F) | 0x20, 0x00}; //set BUSY
                      LNSendPacket(SetStat1, 4); 
                    }
                    byte NullMove[] = {OPC_MOVE_SLOTS, Slot, 0x00, 0x00};
                    LNSendPacket(NullMove, 4); 
                    dpgetSLot = false;  //reset
                   }
                   //update status and notify all devices 
                   #if defined(LnDEB)
                   Debug.print(slot[Slot].LAdr);
                   Debug.print(" Lok config, dir: ");
                   Debug.println((LnPacket->data[6] >> 5) & 0x01);
                   #endif
                                        
                   if ((slot[Slot].Status & B111) == LNLOCO14)
                    dcc.setSpeed14(slot[Slot].LAdr, (LnPacket->data[5] & 0x7F) | ((LnPacket->data[6] << 2) & 0x80));  //
                   else {
                     if ((slot[Slot].Status & B111) == LNLOCO28) 
                       dcc.setSpeed14(slot[Slot].LAdr, (LnPacket->data[5] & 0x7F) | ((LnPacket->data[6] << 2) & 0x80));  //
                     else dcc.setSpeed128(slot[Slot].LAdr, (LnPacket->data[5] & 0x7F) | ((LnPacket->data[6] << 2) & 0x80));  // 
                   }
                   dcc.setFunctions0to4(slot[Slot].LAdr, LnPacket->data[6] & B00011111);
                   dcc.setFunctions5to8(slot[Slot].LAdr, LnPacket->data[10] & 0x0F);            

                   LNGetLocoStatus(Slot);     //request for other devices
                  #endif
                  break;
        }
        case OPC_LOCO_ADR: { //0xBF = Request loco address
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, false);   
                  #endif
                  //add to a SLOT:
                  byte newSlot = LNGetSetLocoSlot((LnPacket->data[1] << 7) | (LnPacket->data[2] & 0x7F), true); //ADR2:7 ms-bits = 0 bei kurzer Adr; ADR:7 ls-bit
                  if (dispatchSlot != 0 && LnPacket->data[1] == 0 && LnPacket->data[2] == 0)
                    newSlot = dispatchSlot;
                  #if defined(LnDEB)
                  Debug.print("get Slot: ");
                  Debug.print(newSlot);
                  #endif  
                  if (newSlot == 0) {
                    //0xB4 = OPC_LONG_ACK No free slot available
                    byte Fail[] = {OPC_LONG_ACK, OPC_LOCO_ADR & 0x7F, 0x00, 0x00};
                    LNSendPacket (Fail, 4);
                    #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                    z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, false);   
                    #endif
                    break;
                  }
                  LNGetLocoStatus(newSlot);
                  break;
        }
        case OPC_MOVE_SLOTS: { //0xBA = Move slot SRC to DST
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, false);   
                  #endif
                  #if defined(LnSLOTSRV) 
                  if (LnPacket->data[1] == 0) {  //SRC = 0
                    //SLOT READ DATA of DISPATCH Slot
                    if (dispatchSlot != 0) {
                      slot[dispatchSlot].Status = slot[dispatchSlot].Status | 0x30;  //IN_USE
                      LNGetLocoStatus(dispatchSlot); //Give slot that was DISPATCHED
                      dispatchSlot = 0;  //reset the Dispatch SLOT
                      break;
                    }
                  }
                  else if (LnPacket->data[1] == LnPacket->data[2]) {  //NULL move
                    //SRC=DEST is set to IN_USE , if legal move -> NULL move
                    slot[LnPacket->data[1]].Status = slot[LnPacket->data[1]].Status | 0x30;  //B00011111;  //IN_USE
                    LNGetLocoStatus(LnPacket->data[1]); 
                    break;
                  }
                  else if (LnPacket->data[2] == 0) {  //DST = 0
                    //DISPATCH Put, mark SLOT as DISPATCH;
                    dispatchSlot = LnPacket->data[1];
                    //RETURN slot status <0xE7> of DESTINATION slot DEST if move legal
                    LNGetLocoStatus(dispatchSlot); 
                    break;
                  }
                  //RETURN Fail LACK code if illegal move <B4>,<3A>,<0>,<chk>
                  byte Fail[] = {OPC_LONG_ACK, OPC_MOVE_SLOTS & 0x7F, 0x00, 0x00};
                  LNSendPacket (Fail, 4);
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, true);
                  #endif
                  break;
                  #endif
        }
        case OPC_RQ_SL_DATA: //Request slot data/status block
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, false);   
                  #endif
                  LNGetLocoStatus(LnPacket->data[1]);
                  break;
        case OPC_LINK_SLOTS: 
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, false);   
                  #endif
                  break;  //Link slot ARG1 to slot ARG2
        case OPC_UNLINK_SLOTS: 
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, false);   
                  #endif
                  break;  //Unlink slot ARG1 from slot ARG2
        case OPC_SLOT_STAT1: 
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, false);   
                  #endif
                  slot[LnPacket->data[1]].Status = LnPacket->data[2];
                  break;  
        case OPC_WR_SL_DATA: {  //Write slot data
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, false);   
                  #endif
                  slot[LnPacket->data[2]].LAdr = (LnPacket->data[9] << 7) | (LnPacket->data[4] & 0x7F);    //ADR2 | ADR
                  slot[LnPacket->data[2]].Status = LnPacket->data[3];  //Save new Status
                                    
                  if ((slot[LnPacket->data[2]].Status & B111) == LNLOCO14)
                    dcc.setSpeed14(slot[LnPacket->data[2]].LAdr, (LnPacket->data[5] & 0x7F) | ((LnPacket->data[6] << 2) & 0x01));  //DIRF & SPD
                  else {
                    if ((slot[LnPacket->data[2]].Status & B111) == LNLOCO28)  
                      dcc.setSpeed28(slot[LnPacket->data[2]].LAdr, (LnPacket->data[5] & 0x7F) | ((LnPacket->data[6] << 2) & 0x01));  //DIRF & SPD
                    else dcc.setSpeed128(slot[LnPacket->data[2]].LAdr, (LnPacket->data[5] & 0x7F) | ((LnPacket->data[6] << 2) & 0x01));  //DIRF & SPD
                  }
                  dcc.setFunctions0to4(slot[LnPacket->data[2]].LAdr, LnPacket->data[6] & B00011111);	//DIRF = - F0 F4 F3 F2 F1
                  dcc.setFunctions5to8(slot[LnPacket->data[2]].LAdr, LnPacket->data[10] & 0x0F);	//SND = - F8 F7 F6 F5
                  //dcc.getLocoStateFull(slot[LnPacket->data[2]].LAdr, false);      //request for other devices
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLocoStateExt (slot[LnPacket->data[2]].LAdr);
                  #endif
                  #if defined(XPRESSNET)
                  XpressNet.setSpeed(slot[LnPacket->data[2]].LAdr, 128, (LnPacket->data[5] & 0x7F) | ((LnPacket->data[6] << 2) & 0x01));
                  XpressNet.setFunc0to4(slot[LnPacket->data[2]].LAdr, LnPacket->data[6] & B00011111);
                  XpressNet.setFunc5to8(slot[LnPacket->data[2]].LAdr, LnPacket->data[10] & 0x0F);
                  XpressNet.ReqLocoBusy(slot[LnPacket->data[2]].LAdr);   //Lok wird nicht von LokMaus gesteuert!
                  #endif
                  //Response:
                  //0=busy/aborted, 1=accepted(OPC_SL_RD_DATA), 0×40=accepted blind(OPC_SL_RD_DATA), 0x7F=not implemented 
                  byte ACK[] = {OPC_LONG_ACK, OPC_WR_SL_DATA & B01111111, 1, 0x00};  
                  LNSendPacket (ACK, 4);    //Send ACK
                  LNGetLocoStatus(LnPacket->data[2]);  //Send OPC_SL_RD_DATA
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, true);   
                  #endif
                  break; 
        }
        case OPC_LOCO_SPD: {    //0SSSSSS
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetLocos_s, false);   
                  #endif
  //                if (LnPacket->data[2] == 0x7F)
  //                  LnPacket->data[2] -= 1;
                  
                  if ((slot[LnPacket->data[1]].Status & B111) == LNLOCO14)
                    dcc.setSpeed14(slot[LnPacket->data[1]].LAdr, (dcc.getLocoDir(slot[LnPacket->data[1]].LAdr) << 7) | LnPacket->data[2]);
                  else {
                    if ((slot[LnPacket->data[1]].Status & B111) == LNLOCO28)
                      dcc.setSpeed28(slot[LnPacket->data[1]].LAdr, (dcc.getLocoDir(slot[LnPacket->data[1]].LAdr) << 7) | LnPacket->data[2]);
                    else dcc.setSpeed128(slot[LnPacket->data[1]].LAdr, (dcc.getLocoDir(slot[LnPacket->data[1]].LAdr) << 7) | LnPacket->data[2]);
                  }
                  //dcc.getLocoStateFull(slot[LnPacket->data[1]].LAdr, false);      //request for other devices
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLocoStateExt (slot[LnPacket->data[1]].LAdr);
                  #endif
                  #if defined(XPRESSNET)
                  XpressNet.setSpeed(slot[LnPacket->data[1]].LAdr, 128, (dcc.getLocoDir(slot[LnPacket->data[1]].LAdr) << 7) | LnPacket->data[2]);
                  XpressNet.ReqLocoBusy(slot[LnPacket->data[1]].LAdr);   //Lok wird nicht von LokMaus gesteuert!
                  #endif
                  break;    
        }  
        case OPC_LOCO_DIRF: { //0,0,DIR,F0,F4,F3,F2,F1
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetLocos_s, false);   
                  #endif
                  byte lokspeed = dcc.getLocoSpeed(slot[LnPacket->data[1]].LAdr);  //lese aktuelle Geschwindigkeit
                  bitWrite(lokspeed, 7, (LnPacket->data[2] >> 5) & 0x01);  //Fahrrichtung

                  if ((slot[LnPacket->data[1]].Status & B111) == LNLOCO14)
                  dcc.setSpeed14(slot[LnPacket->data[1]].LAdr, lokspeed );  //update DIRF in DCC library
                  else { 
                    if ((slot[LnPacket->data[1]].Status & B111) == LNLOCO28)
                       dcc.setSpeed28(slot[LnPacket->data[1]].LAdr, lokspeed );  //update DIRF in DCC library 
                    else dcc.setSpeed128(slot[LnPacket->data[1]].LAdr, lokspeed );  //update DIRF in DCC library
                  }
                  dcc.setFunctions0to4(slot[LnPacket->data[1]].LAdr, LnPacket->data[2] & B00011111);	//- F0 F4 F3 F2 F1
                  
                  //dcc.getLocoStateFull(slot[LnPacket->data[1]].LAdr, false);      //request for other devices
                  
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLocoStateExt (slot[LnPacket->data[1]].LAdr);
                  #endif
                  
                  #if defined(XPRESSNET)
                  XpressNet.setFunc0to4(slot[LnPacket->data[1]].LAdr, LnPacket->data[2] & B00011111);
                  XpressNet.ReqLocoBusy(slot[LnPacket->data[1]].LAdr);   //Lok wird nicht von LokMaus gesteuert!
                  #endif
                  
                  break;
        }
        case OPC_LOCO_SND: { //0,0,0,0,F8,F7,F6,F5 
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetLocos_s, false);   
                  #endif
                  dcc.setFunctions5to8(slot[LnPacket->data[1]].LAdr, LnPacket->data[2]);	//- F8 F7 F6 F5
                  
                  //dcc.getLocoStateFull(slot[LnPacket->data[1]].LAdr, false);      //request for other devices
                  
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLocoStateExt (slot[LnPacket->data[1]].LAdr);
                  #endif
                  
                  #if defined(XPRESSNET)
                  XpressNet.setFunc5to8(slot[LnPacket->data[1]].LAdr, LnPacket->data[2]);
                  XpressNet.ReqLocoBusy(slot[LnPacket->data[1]].LAdr);   //Lok wird nicht von LokMaus gesteuert!
                  #endif
                  
                  break; 
        }
        case OPC_IMM_PACKET: { //Digitrax OPC_LOCO_F912 = Functions 9-12
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetLocos_s, false);   
                  #endif
                  byte ACK[] = {OPC_LONG_ACK, OPC_IMM_PACKET & 0x7F, 0x00, 0x00};  //busy
                  LNSendPacket (ACK, 4);    //Send ACK 
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetLocos_s, true);            
                  #endif
                  break;
        }
        case 0xA3: {  //0,0,0,0,F12,F11,F10,F9 by Uhlenbrock
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetLocos_s, false);   
                  #endif
                  dcc.setFunctions9to12(slot[LnPacket->data[1]].LAdr, LnPacket->data[2]);  //- F12 F11 F10 F9
                  
                  //dcc.getLocoStateFull(slot[LnPacket->data[1]].LAdr, false);      //request for other devices
                  
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLocoStateExt (slot[LnPacket->data[1]].LAdr);
                  #endif
                  
                  #if defined(XPRESSNET)
                  XpressNet.setFunc9to12(slot[LnPacket->data[1]].LAdr, LnPacket->data[2]);
                  XpressNet.ReqLocoBusy(slot[LnPacket->data[1]].LAdr);   //Lok wird nicht von LokMaus gesteuert!
                  #endif
                  
                  break; 
        }
        case OPC_UHLI_FUN: {  //Function 13-28 by Uhlenbrock (0xD4)
                  if (LnPacket->data[1] == 0x20) {
                    #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                    z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetLocos_s, false);   
                    #endif
                    byte Func = 0x00;
                    if (LnPacket->data[3] == 0x07) { //Arg3
                      if ((LnPacket->data[4] & 0x10) != 0) 
                        Func |= B0001; //F9
                      if ((LnPacket->data[4] & 0x20) != 0) 
                        Func |= B0010; //F10
                      if ((LnPacket->data[4] & 0x40) != 0) 
                        Func |= B0100; //F11  
                      dcc.setFunctions9to12(slot[LnPacket->data[2]].LAdr, Func);	//- F12 F11 F10 F9
                      #if defined(XPRESSNET)
                      XpressNet.setFunc9to12(slot[LnPacket->data[2]].LAdr, Func);
                      #endif
                    }
                    if (LnPacket->data[3] == 0x08) {  //for F13 to F19
                      Func = (LnPacket->data[4] & 0x7f) | (dcc.getFunktion13to20(slot[LnPacket->data[2]].LAdr) & 0x80);
                      dcc.setFunctions13to20(slot[LnPacket->data[2]].LAdr, Func);  //F20 to F13
                      #if defined(XPRESSNET)
                      XpressNet.setFunc13to20(slot[LnPacket->data[2]].LAdr, Func);
                      #endif
                    }
                    if (LnPacket->data[3] == 0x09) { //for F21 to F27
                      Func = (LnPacket->data[4] & 0x7f) | (dcc.getFunktion21to28(slot[LnPacket->data[2]].LAdr) & 0x80);
                      dcc.setFunctions21to28(slot[LnPacket->data[2]].LAdr, Func);  //F28 to F21
                      #if defined(XPRESSNET)
                      XpressNet.setFunc21to28(slot[LnPacket->data[2]].LAdr, Func);
                      #endif
                    }
                    if (LnPacket->data[3] == 0x05) { //for F28 and F20
                      Func = ((LnPacket->data[4] << 2) & 0x80) | (dcc.getFunktion13to20(slot[LnPacket->data[2]].LAdr) & 0x7f);
                      dcc.setFunctions13to20(slot[LnPacket->data[2]].LAdr, Func);  //F20 to F13
                      #if defined(XPRESSNET)
                      XpressNet.setFunc13to20(slot[LnPacket->data[2]].LAdr, Func);
                      #endif
                      Func = ((LnPacket->data[4] << 1) & 0x80) | (dcc.getFunktion21to28(slot[LnPacket->data[2]].LAdr) & 0x7f);
                      dcc.setFunctions21to28(slot[LnPacket->data[2]].LAdr, Func);  //F28 to F21
                      #if defined(XPRESSNET)
                      XpressNet.setFunc21to28(slot[LnPacket->data[2]].LAdr, Func);
                      #endif
                    }
                    #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                    z21.setLocoStateExt (slot[LnPacket->data[2]].LAdr);
                    #endif
                    //dcc.getLocoStateFull(slot[LnPacket->data[2]].LAdr, false);      //request for other devices

                    #if defined(XPRESSNET)
                    XpressNet.ReqLocoBusy(slot[LnPacket->data[2]].LAdr);   //Lok wird nicht von LokMaus gesteuert!
                    #endif
                  }
                  break;    
        }
        case OPC_SW_STATE: {  //Request state of switch. 
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetSwitches_s, false);   
                  #endif
                  //dcc.getBasicAccessoryInfo(Address+inc)
                  #if defined(LnSLOTSRV)
                  //byte LOPC = LnPacket->data[0] & 0x7F;  //Kopie der Kommando-Codes. Das 7. Bit wird 0 gesetzt 
                  byte ACK[] = {OPC_LONG_ACK, 0x00, 0x00, 0x00};  //Fail!!
                  ACK[1] = LnPacket->data[0] & B01111111;
                  LNSendPacket (ACK, 4);    //Send ACK
                  
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetSwitches_s, true);   
                  #endif
                  
                  #endif
                  break;
        }
        case OPC_SW_ACK: { //Request switch with acknoledge function.
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetSwitches_s, false);   
                  #endif
                  dcc.setBasicAccessoryPos(LnPacket->data[1] | ((LnPacket->data[2] & 0x0F) << 7),(LnPacket->data[2] >> 5) & 0x01, (LnPacket->data[2] >> 4) & 0x01);  //Adr, State, on/off
                  #if defined(LnSLOTSRV)
                  //byte LOPC = LnPacket->data[0] & B01111111;  //Kopie der Kommando-Codes. Das 7. Bit wird 0 gesetzt 
                  byte ACK[] = {OPC_LONG_ACK, 0x00, 0x7F, 0x00};  //Succsess
                  ACK[1] = LnPacket->data[0] & 0x7F;
                  LNSendPacket (ACK, 4);    //Send ACK
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetSwitches_s, true);   
                  #endif
                  #endif
                  break;
        }        
        case OPC_SW_REQ: //Request switch function 
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetSwitches_s, false);   
                  #endif
                  dcc.setBasicAccessoryPos(LnPacket->data[1] | ((LnPacket->data[2] & 0x0F) << 7),(LnPacket->data[2] >> 5) & 0x01, (LnPacket->data[2] >> 4) & 0x01);  //Adr, State, on/off
                  #if defined(XPRESSNET)
                  XpressNet.SetTrntPos(LnPacket->data[1] | ((LnPacket->data[2] & 0x0F) << 7),(LnPacket->data[2] >> 5) & 0x01, (LnPacket->data[2] >> 4) & 0x01); //Adr, state, active
                  #endif                  
                  break;
        case OPC_SW_REP: { //Turnout sensor state report 
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, false);   
                  //LnPacket->data[1] = 0,A6,A5,A4,A3,A2,A1,A0 
                  //LnPacket->data[2] =   0,X,I,L,A10,A9,A8,A7
                  byte Rdata[4];
                  Rdata[0] = 0x01; //Typ
                  Rdata[1] = LnPacket->data[2] & B1111;  //Adress A10-A8
                  Rdata[2] = (LnPacket->data[1] << 1) | ((LnPacket->data[2] >> 5) & 0x01);  //A7-A0
                  Rdata[3] = (LnPacket->data[2] >> 4) & 0x01; //L, Rückmelde-Zustand
                  z21.setLNDetector(Rdata, 4);
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, true);   
                  #endif
                  break;         
        }
        case OPC_INPUT_REP: { //0xB2 = Besetztmelder - LAN_LOCONET_DETECTOR
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, false);   
                  #endif
                  //LnPacket->data[1] = 0,A6,A5,A4,A3,A2,A1,A0 
                  //LnPacket->data[2] = 	0,X,I,L,A10,A9,A8,A7
                  byte Rdata[4];
                  Rdata[0] = 0x01; //Typ
                  Rdata[1] = LnPacket->data[2] & B1111;  //Adress A10-A8
                  Rdata[2] = (LnPacket->data[1] << 1) | ((LnPacket->data[2] >> 5) & 0x01);  //A7-A0
                  Rdata[3] = (LnPacket->data[2] >> 4) & 0x01; //L, Rückmelde-Zustand
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNDetector(Rdata, 4);
                  #endif
                  #if defined(REPORT)
                  Debug.print("LN Sensor:");
                  //Debug.print(((LnPacket->data[1] << 1) | ((LnPacket->data[2] & B1111) << 8) | ((LnPacket->data[2] >> 5) & 0x01)) + 1);
                  Debug.print(word(Rdata[1],Rdata[2])+1);
                  Debug.println(((LnPacket->data[2] >> 4) & 0x01) ? "=on" : "=off");
                  #endif
                  break; }
        case OPC_MULTI_SENSE: {
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, false);   
                  #endif
                  byte Rdata[4];
                  Rdata[0] = LnPacket->data[1]; //Type
                  Rdata[1] = LnPacket->data[3]; //Adr
                  Rdata[1] = LnPacket->data[4]; //Adr
                  Rdata[3] = LnPacket->data[2]; //zone and section
                  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                  z21.setLNDetector(Rdata, 4);  
                  #endif
                  break; }
        //Zustand Gleisspannung
        case OPC_GPOFF: {
                        #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                        z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, false);   
                        #endif
                        if (Railpower != csTrackVoltageOff)
                          globalPower(csTrackVoltageOff); 
                        break; }
        case OPC_GPON:  {
                        #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                        z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, false);   
                        #endif
                        if (Railpower != csNormal) 
                           globalPower(csNormal); 
                        break; }
        case OPC_IDLE:  {
                        #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
                        z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, false);   
                        #endif
                        if (Railpower != csEmergencyStop) 
                          globalPower(csEmergencyStop); 
                        break; }
        #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
        default: z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, false);   
        #endif                
      }
   }
}

//--------------------
#endif
