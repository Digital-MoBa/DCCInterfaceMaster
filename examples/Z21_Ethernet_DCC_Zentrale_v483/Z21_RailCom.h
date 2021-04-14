//Z21 DCC RailCom Decoder
//Dekodierung globaler Railcomdaten am Serial3 (MEGA only!)
//
//Copyright (c) by Philipp Gahtow, year 2017

//--------------------------------------------------------------
#if defined(DCCGLOBALDETECTOR) && defined(DCC)

byte RailcomCVAdr = 0;  //init global RailCom detector for POM

//--------------------------------------------------------------
#if defined(MEGA_MCU)
/*
void notifyCVPOMRead(uint16_t Adr, uint8_t value) {
  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
    z21.setCVPOMBYTE(Adr, value);
  #endif
}
*/

#define RCSerial Serial3  //Serial Port that we will listen to for RailCom Data

//--------------------------------------------------------------
byte RailComGetCV = false;  //try to receive a CV value
byte RailComReadFirst = true;
byte RailComReadData = 0; //Save the first 2 bit data
byte RailComReadLastCV = 0xFF; //Save the last read CV value
byte RailComCVRead = false; //receive a CV value
byte RailComCVTime = 0; //cycle time
uint16_t RailComReadAdr = 0;  //Adress read over RailCom

//--------------------------------------------------------------
//init the Serial interface for receiving RailCom
void RailComSetup(void) {
  RCSerial.begin(250000);    //Read Railcom Message at 250KBit
}

//--------------------------------------------------------------
//Decoding RailCom Received Data
byte RailComDecodeInData (void) {
  switch (RCSerial.read()) {
    case B10101100: return 0x00;
    case B10110010: return 0x10; 
    case B01010110: return 0x20;
    case B11000110: return 0x30;
    case B10101010: return 0x01;
    case B10110100: return 0x11;
    case B01001110: return 0x21;
    case B11001100: return 0x31;
    case B10101001: return 0x02;
    case B10111000: return 0x12;
    case B01001101: return 0x22;
    case B01111000: return 0x32;
    case B10100101: return 0x03;
    case B01110100: return 0x13;
    case B01001011: return 0x23;
    case B00010111: return 0x33;
    case B10100011: return 0x04;
    case B01110010: return 0x14;
    case B01000111: return 0x24;
    case B00011011: return 0x34;
    case B10100110: return 0x05;
    case B01101100: return 0x15;
    case B01110001: return 0x25;
    case B00011101: return 0x35;
    case B10011100: return 0x06;
    case B01101010: return 0x16;
    case B11101000: return 0x26;
    case B00011110: return 0x36;
    case B10011010: return 0x07;
    case B01101001: return 0x17;
    case B11100100: return 0x27;
    case B00101110: return 0x37;
    case B10011001: return 0x08;
    case B01100101: return 0x18;
    case B11100010: return 0x28;
    case B00110110: return 0x38;
    case B10010101: return 0x09;
    case B01100011: return 0x19;
    case B11010001: return 0x29;
    case B00111010: return 0x39;
    case B10010011: return 0x0A;
    case B01100110: return 0x1A;
    case B11001001: return 0x2A;
    case B00100111: return 0x3A;
    case B10010110: return 0x0B;
    case B01011100: return 0x1B;
    case B11000101: return 0x2B;
    case B00101011: return 0x3B;
    case B10001110: return 0x0C;
    case B01011010: return 0x1C;
    case B11011000: return 0x2C;
    case B00101101: return 0x3C;
    case B10001101: return 0x0D;
    case B01011001: return 0x1D;
    case B11010100: return 0x2D;
    case B00110101: return 0x3D;
    case B10001011: return 0x0E;
    case B01010101: return 0x1E;
    case B11010010: return 0x2E;
    case B00111001: return 0x3E;
    case B10110001: return 0x0F;
    case B01010011: return 0x1F;
    case B11001010: return 0x2F;
    case B00110011: return 0x3F;
    case 0x0F: return 0xF0;   //ACK
  }
  return 0xFF;
}

//--------------------------------------------------------------
//Check if we receive data?
void RailComRead(void) {
  if (RailComGetCV) { //need to read data?
    RailComCVTime++;
    while (RCSerial.available()) {  //is there serial data in buffer?
      byte data = RailComDecodeInData();   //read Railcom Data and decode Railcom Data
      if (data < 0x40) {    //gültige Railcom Message empfangen
            if (RailComReadFirst) {
              RailComReadData = data;   //Save first Byte of Data
              if ((data >> 2) < 3)  //app:pom, app:adr_low, app:adr_high -> read only 12 Bit!
                RailComReadFirst = false;
            }
            else {
              RailComReadFirst = true;
              byte RailComID = RailComReadData >> 2; //Save ID
              RailComReadData = (RailComReadData << 6) | data;    //first 2 Bit and add next 6 Bit
              if (RailComID == 0) { //app:pom
                  RailComCVTime = 25; //Reset Timer!
                  RailComReadLastCV = RailComReadData;
                  RailComCVRead = true;
              }
              else if (RailComID == 1) { //app:adr_high
                RailComReadAdr = (RailComReadData << 8) | (RailComReadAdr & 0xFF);
              }
              else if (RailComID == 2) {  //app:adr_low
                RailComReadAdr = (RailComReadAdr & 0xFF00) | RailComReadData;
              }
            }
      }
      else RailComReadFirst = true;
    }
    if (RailComCVTime == 0) {  //TIMEOUT?
      RailComGetCV = false; //Stop reading! ERROR!
      if (RailComCVRead == true) {
        RailComCVRead = false;
        #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
        z21.setCVPOMBYTE(RailcomCVAdr, RailComReadLastCV);
        #endif
        #if defined(RCDEB)
        Debug.print(" A");
        Debug.print(RailComReadAdr);
        Debug.print(" value: ");
        Debug.println(RailComReadLastCV);
        #endif
      }
      else {
        #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
        z21.setCVNack();  //Antwort: LAN_X_CV_NACK
        #endif
        #if defined(RCDEB)
        Debug.println(" RC ERROR");
        #endif
      }
    }
  }
}

//--------------------------------------------------------------
//Start Reading Data!
void RailComStart(void) {
  while(RCSerial.available()){RCSerial.read();}  //remove all rubish data
  RailComGetCV = true;  //start reading RailCom Data
  RailComReadLastCV = 0xFF;  //Reset the last Value
  RailComCVTime = 0;  //Reset the cycle time
  RailComReadAdr = 0;
  RailComRead();  //Try to read data...
}

#endif  //End MEGA Serial

//--------------------------------------------------------------
#if defined(RAILCOMI2C)
//--------------------------------------------------------------
// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void RailComReceiveEvent(int howMany) {
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
  }
  byte value = Wire.read();    // receive byte
  
  #if defined(RCDEB)
  Debug.print(" value: ");
  Debug.println(value);
  #endif
  #if defined(LAN) || defined(WIFI) || defined(ESP_WIFI)
  z21.setCVPOMBYTE(RailcomCVAdr, value);
  #endif
}
#endif
//--------------------------------------------------------------
#endif
