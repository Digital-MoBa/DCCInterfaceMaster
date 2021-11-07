/*
* DCC Waveform Generator
*
* modified by Philipp Gahtow
* Copyright 2021
* digitalmoba@arcor.de, http://pgahtow.de
*
*/

#include "DCCPacketScheduler.h"
#include "DCCHardware.h"
#include "DDCHardware_config.h"

#if defined(GLOBALRAILCOMREADER)
#include <avr/interrupt.h>
#endif

#if defined(__SAM3X8E__)	//Arduino DUE
#include <DueFlashStorage.h>
DueFlashStorage DueFlash;
#define FSTORAGE 	DueFlash
#define FSTORAGEMODE write

#elif defined(ESP8266) || defined(ESP32) //ESP8266 or ESP32
#include "z21nvs.h"
z21nvsClass EEPROMDCC;
#define FSTORAGE EEPROMDCC
#define FSTORAGEMODE write

#else
// AVR based Boards follows
#include <EEPROM.h>
#define FSTORAGE 	EEPROM
#define FSTORAGEMODE update
#endif

#if defined(ESP32)
extern portMUX_TYPE timerMux;	
#endif

extern volatile bool get_next_packet; 

/// The Pin where the DCC Waveform comes out.
extern uint8_t DCCPin;
extern uint8_t DCCPin2;
extern bool RailCom;
extern uint8_t DCCS88Pin;
/// The currently queued packet to be put on the rails. Default is a reset packet.
extern uint8_t current_packet[6];
/// is in Service Mode:
extern volatile uint8_t current_packet_service;
/// How many data uint8_ts in the queued packet?
extern volatile uint8_t current_packet_size;

volatile uint8_t current_load_pin = 0;	//Pin where current loadis detected
volatile uint8_t current_ack_read = false;	//ack is detected
volatile uint16_t current_cv = 0;	//cv that we are working on
volatile uint8_t current_cv_value = 0;	//value that is read
volatile uint8_t current_cv_bit = 0xFF;	//bit that will be read - 0xFF = ready, nothing to read!
volatile uint8_t ack_received_now = false;  //ACK is send by the decoder
volatile long ack_received_time = 0;	//micro time that the ACK start
volatile long ack_monitor_time = 0;		//time when a ACK packet is prepared!

#if defined(ESP32)
extern hw_timer_t * timer;
#endif

#if defined(GLOBALRAILCOMREADER)
//DCCPacketScheduler *DCCPacketScheduler::active_object = 0;	//Static handle object for interrupt
#endif

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////
  
DCCPacketScheduler::DCCPacketScheduler(void) : /*default_speed_steps(128),*/ /*last_packet_address(255),*/ packet_counter(1)
{
  e_stop_queue.setup(E_STOP_QUEUE_SIZE);	//just to send once for set repeat circle
//  high_priority_queue.setup(HIGH_PRIORITY_QUEUE_SIZE);
//  low_priority_queue.setup(LOW_PRIORITY_QUEUE_SIZE);
  repeat_queue.setup(REPEAT_QUEUE_SIZE);  //all repeat packets are inserted here, repeat first with priority!
//NEW
  periodic_refresh_queue.setup(PERIODIC_REFRESH_QUEUE_SIZE);	//collect all packtes there are permanent to repeat

  ops_programmming_queue.setup(PROG_QUEUE_SIZE); //for CV programming only
}

void DCCPacketScheduler::setup(uint8_t pin, uint8_t pin2, uint8_t steps, uint8_t format, uint8_t power) //for any post-constructor initialization
{
	loadEEPROMconfig();	//load the configuration
	
	DCCPin = pin;	//set DCC Waveform pin
	DCCPin2 = pin2;	//set inverted DCC Waveform pin2
	DCCS88Pin = 0xFF;	//disable per default
	
	setup_DCC_waveform_generator();	//Timer neu configurieren
	
	#if defined(__SAM3X8E__)
	TC_Start(DCC_ARM_TC_TIMER, DCC_ARM_TC_CHANNEL);
	#elif defined(ESP8266) 	//ESP8266
	timer1_enable(DCC_ESP_TIMER_DIV, DCC_ESP_TIMER_SET, DCC_ESP_TIMER_LOOP);
	#endif	
	
	setpower(power, true);	//DCC signal on (active) and inform other over the new power state!
	
	slotFullNext = 0;	//don't override, start with free slots
	TrntFormat = format;	//The way BasicAccessory Messages Addressing works (Intellbox/ROCO/etc)
	DCCdefaultSteps = steps;

	//Following RP 9.2.4, begin by putting 20 reset packets and 10 idle packets on the rails.
	//reset packet: address 0x00, data 0x00, XOR 0x00; S 9.2 line 75
	opsDecoderReset(20);	//send first a Reset Packet

	//idle packet: address 0xFF, data 0x00, XOR 0xFF; S 9.2 line 90
	//Automatically send idle packet until first action!

	#if defined(GLOBALRAILCOMREADER)
		Serial3.begin(250000);
		POMCVAdr = 0xFFFF;
	#endif
}

//extra DCC signal for S88/LocoNet without Shutdown and Railcom
void DCCPacketScheduler::enable_additional_DCC_output(uint8_t pin)
{
	DCCS88Pin = pin;		//set PIN for S88/LocoNet true DCC output
	setup_DCC_waveform_generator();	//Timer neu configurieren
}

void DCCPacketScheduler::disable_additional_DCC_output(void)
{
	enable_additional_DCC_output(0xFF); //disable - no PIN set
}

void DCCPacketScheduler::loadEEPROMconfig(void)
{
	if (FSTORAGE.read(EEPROMRailCom) > 1)
		FSTORAGE.FSTORAGEMODE(EEPROMRailCom,0x01);	//Default activ
	RailCom = FSTORAGE.read(EEPROMRailCom);	//define if railcom cutout is active	
	
	if ((FSTORAGE.read(EEPROMRSTsRepeat) > 64) | (FSTORAGE.read(EEPROMRSTcRepeat) > 64)) {
		FSTORAGE.FSTORAGEMODE(EEPROMProgRepeat,OPS_MODE_PROGRAMMING_REPEAT);
		FSTORAGE.FSTORAGEMODE(EEPROMRSTsRepeat,RESET_START_REPEAT);
		FSTORAGE.FSTORAGEMODE(EEPROMRSTcRepeat,RESET_CONT_REPEAT);
	}
	
	#if defined(ESP8266) || defined(ESP32) //ESP8266 or ESP32
	FSTORAGE.commit();
	#endif
	
	ProgRepeat = FSTORAGE.read(EEPROMProgRepeat);	//Repaet for Packet Programming
	RSTsRepeat = FSTORAGE.read(EEPROMRSTsRepeat);	//Repaet for Reset start Packet
	RSTcRepeat = FSTORAGE.read(EEPROMRSTcRepeat);	//Repaet for Reset contingue Packet
}

//set the power for the dcc signal
void DCCPacketScheduler::setpower(uint8_t state, bool notify)
{
	if (railpower != state) {
		railpower = state;	//save the state of the railpower
		if (state == OFF || state == SHORT) {

			DCC_stop_output_signal();	//RailPower-Signal generate OFF
			
		}
		else {
			DCC_run_output_signal(); 	//generate RailPower-Signal
		}
		
		if (notifyRailpower && notify)
			notifyRailpower(railpower);
	}
}

//get the actual state of power
byte DCCPacketScheduler::getpower(void)
{
	return railpower;	
}

//to de-/activate RailCom output
void DCCPacketScheduler::setrailcom(bool rc) 
{
	RailCom = rc;
}	

//return the State of RailCom output
bool DCCPacketScheduler::getrailcom(void) 
{
	return RailCom;
}
	
/*
//helper functions
void DCCPacketScheduler::repeatPacket(DCCPacket *p)
{
  switch(p->getKind())
  {
    case idle_packet_kind:
    case e_stop_packet_kind: //e_stop packets automatically repeat without having to be put in a special queue
      break;
    case speed_packet_kind: //speed packets go to the periodic_refresh queue
    case function_packet_1_kind: //all other packets go to the repeat_queue
    case function_packet_2_kind: //all other packets go to the repeat_queue
    case function_packet_3_kind: //all other packets go to the repeat_queue
	case function_packet_4_kind: //all other packets go to the repeat_queue
	case function_packet_5_kind: //all other packets go to the repeat_queue
		periodic_refresh_queue.insertPacket(p);
		break;
    case accessory_packet_kind:
    case reset_packet_kind:
    case ops_mode_programming_kind:
    case other_packet_kind:
    default:
      repeat_queue.insertPacket(p);
  }
}
*/

//for enqueueing packets

//setSpeed* functions:
//new_speed contains the speed and direction.
// a value of 0 = estop
// a value of 1/-1 = stop
// a value >1 (or <-1) means go.
// valid non-estop speeds are in the range [1,127] / [-127,-1] with 1 = stop

bool DCCPacketScheduler::setSpeed(uint16_t address, uint8_t speed)
{
	//set Loco to speed with default settings!
  switch(DCCdefaultSteps)
  {
    case 14:
      return(setSpeed14(address, speed));
    case 28:
      return(setSpeed28(address, speed));
    case 128:
      return(setSpeed128(address, speed));
  }
  return false; //invalid number of steps specified.
}

bool DCCPacketScheduler::setSpeed14(uint16_t address, uint8_t speed)
{
	if (address == 0)	//check if Adr is ok?
		return false;

	byte slot = LokStsgetSlot(address);
	LokDataUpdate[slot].speed = speed;	//write Dir and Speed into register to SAVE
	if ((LokDataUpdate[slot].adr >> 14) != DCC14)  //0=>14steps, write speed steps into register
		LokDataUpdate[slot].adr = (LokDataUpdate[slot].adr & 0x3FFF) | (DCC14 << 14);		
  
	uint8_t speed_data_uint8_ts[] = {0x40};		//speed indecator
	/*
    if (speed == 1) //estop!
		//return eStop(address);//
		speed_data_uint8_ts[0] |= 0x01; //estop
    else if (speed == 0) //regular stop!
		speed_data_uint8_ts[0] |= 0x00; //stop
    else //movement
		speed_data_uint8_ts[0] |= map(speed, 2, 127, 2, 15); //convert from [2-127] to [1-14]
    speed_data_uint8_ts[0] |= (0x20 * bitRead(speed, 7)); //flip bit 3 to indicate direction;
	*/
	speed_data_uint8_ts[0] |= speed & 0x1F;			//5 Bit Speed
	speed_data_uint8_ts[0] |= (speed & 0x80) >> 2;	//Dir
	
    DCCPacket p(address);
    p.addData(speed_data_uint8_ts,1);

    p.setRepeat(SPEED_REPEAT);
  
    p.setKind(speed_packet_kind);  

    //speed packets get refreshed indefinitely, and so the repeat doesn't need to be set.
    //speed packets go to the high proirity queue
    //return(high_priority_queue.insertPacket(&p));
	if (railpower == ESTOP)	//donot send to rails now!
		return periodic_refresh_queue.insertPacket(&p);
	return repeat_queue.insertPacket(&p);
}

bool DCCPacketScheduler::setSpeed28(uint16_t address, uint8_t speed)
{
	if (address == 0)	//check if Adr is ok?
		return false;

  byte slot = LokStsgetSlot(address);
  LokDataUpdate[slot].speed = speed;		// speed & B01111111 + Dir;	//write into register to SAVE
  if ((LokDataUpdate[slot].adr >> 14) != DCC28)	//2=>28steps, write into register
	LokDataUpdate[slot].adr = (LokDataUpdate[slot].adr & 0x3FFF) | (DCC28 << 14);

  uint8_t speed_data_uint8_ts[] = {0x40};	//Speed indecator
  /*
  if(speed == 1) //estop!
    //return eStop(address);//
	speed_data_uint8_ts[0] |= 0x01; //estop
  else if (speed == 0) //regular stop!
    speed_data_uint8_ts[0] |= 0x00; //stop
  else //movement
  {
    speed_data_uint8_ts[0] |= map(speed, 2, 127, 2, 0x1F); //convert from [2-127] to [2-31]  
    //most least significant bit has to be shufled around
    speed_data_uint8_ts[0] = (speed_data_uint8_ts[0]&0xE0) | ((speed_data_uint8_ts[0]&0x1F) >> 1) | ((speed_data_uint8_ts[0]&0x01) << 4);
  }
  speed_data_uint8_ts[0] |= (0x20 * bitRead(speed, 7)); //flip bit 3 to indicate direction;
  */
  speed_data_uint8_ts[0] |= speed & 0x1F;			//5 Bit Speed
  speed_data_uint8_ts[0] |= (speed & 0x80) >> 2;	//Dir
	
  DCCPacket p(address);
  p.addData(speed_data_uint8_ts,1);
  
  p.setRepeat(SPEED_REPEAT);
  
  p.setKind(speed_packet_kind);
  
  //speed packets get refreshed indefinitely, and so the repeat doesn't need to be set.
  //speed packets go to the high proirity queue
  //return(high_priority_queue.insertPacket(&p));
  if (railpower == ESTOP)	//donot send to rails now!
	  return periodic_refresh_queue.insertPacket(&p);
  return repeat_queue.insertPacket(&p);
}

bool DCCPacketScheduler::setSpeed128(uint16_t address, uint8_t speed)
{
	if (address == 0) {
	//	Serial.println("ERROR ADR0");
		return false;
	}
	byte slot = LokStsgetSlot(address);
	LokDataUpdate[slot].speed = speed;	//write Speed and Dir into register to SAVE
	if ((LokDataUpdate[slot].adr >> 14) != DCC128) //3=>128steps, write into register
		LokDataUpdate[slot].adr = (LokDataUpdate[slot].adr & 0x3FFF) | (DCC128 << 14);

	uint8_t speed_data_uint8_ts[] = { 0x3F, 0x00 };

//	if (speed == 1) //estop!
//		return eStop(address);//speed_data_uint8_ts[1] |= 0x01; //estop
	//else 
		speed_data_uint8_ts[1] = speed; //no conversion necessary.

	//why do we get things like this?
	// 03 3F 16 15 3F (speed packet addressed to loco 03)
	// 03 3F 11 82 AF  (speed packet addressed to loco 03, speed hex 0x11);
	DCCPacket p(address);
	p.addData(speed_data_uint8_ts, 2);

	p.setRepeat(SPEED_REPEAT);

	p.setKind(speed_packet_kind);

	//speed packets get refreshed indefinitely, and so the repeat doesn't need to be set.
	//speed packets go to the high proirity queue

	//return(high_priority_queue.insertPacket(&p));
	if (railpower == ESTOP)	//donot send to rails now!
		return periodic_refresh_queue.insertPacket(&p);
	return repeat_queue.insertPacket(&p);
}

//--------------------------------------------------------------------------------------------
//Lokfunktion setzten
void DCCPacketScheduler::setLocoFunc(uint16_t address, uint8_t type, uint8_t fkt)
{			//type => 0 = AUS; 1 = EIN; 2 = UM; 3 = error
	bool fktbit = 0;	//neue zu ändernde fkt bit
	if (type == 1)	//ein
		fktbit = 1;
	byte Slot = LokStsgetSlot(address);
	//zu änderndes bit bestimmen und neu setzten:
	if (fkt <= 4) {
		byte func = LokDataUpdate[Slot].f0 & 0x1F;	//letztes Zustand der Funktionen 000 F0 F4..F1
		if (type == 2) { //um
			if (fkt == 0)
				fktbit = !(bitRead(func, 4));
			else fktbit = !(bitRead(func, fkt - 1));
		}
		if (fkt == 0)
			bitWrite(func, 4, fktbit);
		else bitWrite(func, fkt - 1, fktbit);
		//Daten senden:
		setFunctions0to4(address, func);	//func = 0 0 0 F0 F4 F3 F2 F1
	}
	else if ((fkt >= 5) && (fkt <= 8)) {
		byte funcG2 = LokDataUpdate[Slot].f1 & 0x0F;	//letztes Zustand der Funktionen 0000 F8..F5
		if (type == 2) //um
			fktbit = !(bitRead(funcG2, fkt - 5));
		bitWrite(funcG2, fkt - 5, fktbit);
		//Daten senden:
		setFunctions5to8(address, funcG2);	//funcG2 = 0 0 0 0 F8 F7 F6 F5
	}
	else if ((fkt >= 9) && (fkt <= 12)) {
		byte funcG3 = LokDataUpdate[Slot].f1 >> 4;	//letztes Zustand der Funktionen 0000 F12..F9
		if (type == 2) //um
			fktbit = !(bitRead(funcG3, fkt - 9));
		bitWrite(funcG3, fkt - 9, fktbit);
		//Daten senden:
		setFunctions9to12(address, funcG3); 	//funcG3 = 0 0 0 0 F12 F11 F10 F9
	}
	else if ((fkt >= 13) && (fkt <= 20)) {
		byte funcG4 = LokDataUpdate[Slot].f2;
		if (type == 2) //um
			fktbit = !(bitRead(funcG4, fkt - 13));
		bitWrite(funcG4, fkt - 13, fktbit);
		//Daten senden:
		setFunctions13to20(address, funcG4);	//funcG4 = F20 F19 F18 F17 F16 F15 F14 F13
	}
	else if ((fkt >= 21) && (fkt <= 28)) {
		byte funcG5 = LokDataUpdate[Slot].f3;
		if (type == 2) //um
			fktbit = !(bitRead(funcG5, fkt - 21));
		bitWrite(funcG5, fkt - 21, fktbit);
		//Daten senden:
		setFunctions21to28(address, funcG5);	//funcG5 = F28 F27 F26 F25 F24 F23 F22 F21
	}
	//getLocoStateFull(address, true);	//Alle aktiven Geräte Senden!
}

bool DCCPacketScheduler::setFunctions0to4(uint16_t address, uint8_t functions)
{
	if (address == 0)	//check if Adr is ok?
		return false;

  DCCPacket p(address);
  uint8_t data[] = { 0x80 };
  
  //Obnoxiously, the headlights (F0, AKA FL) are not controlled
  //by bit 0, but in DCC via bit 4. !
  data[0] |= functions & 0x1F;		//new - normal way of DCC! F0, F4, F3, F2, F1

  p.addData(data,1);
  p.setKind(function_packet_1_kind);
  p.setRepeat(FUNCTION_REPEAT);

  LokDataUpdate[LokStsgetSlot(address)].f0 = functions & 0x1F;	//write into register to SAVE

  //return low_priority_queue.insertPacket(&p);
  return repeat_queue.insertPacket(&p);
}


bool DCCPacketScheduler::setFunctions5to8(uint16_t address, uint8_t functions)
{
	if (address == 0)	//check if Adr is ok?
		return false;

  DCCPacket p(address);
  uint8_t data[] = { 0xB0 };
  data[0] |= functions & 0x0F;
  
  p.addData(data,1);
  p.setKind(function_packet_2_kind);
  p.setRepeat(FUNCTION_REPEAT);

  LokDataUpdate[LokStsgetSlot(address)].f1 = (LokDataUpdate[LokStsgetSlot(address)].f1 | 0x0F) & (functions | 0xF0);	//write into register to SAVE

  //return low_priority_queue.insertPacket(&p);
  return repeat_queue.insertPacket(&p);
}

bool DCCPacketScheduler::setFunctions9to12(uint16_t address, uint8_t functions)
{
	if (address == 0)	//check if Adr is ok?
		return false;

  DCCPacket p(address);
  uint8_t data[] = { 0xA0 };
  //least significant four functions (F5--F8)
  data[0] |= functions & 0x0F;
  
  p.addData(data,1);
  p.setKind(function_packet_3_kind);
  p.setRepeat(FUNCTION_REPEAT);

  LokDataUpdate[LokStsgetSlot(address)].f1 = (LokDataUpdate[LokStsgetSlot(address)].f1 | 0xF0) & ((functions << 4) | 0x0F);	//write into register to SAVE

  //return low_priority_queue.insertPacket(&p);
  return repeat_queue.insertPacket(&p);
}

bool DCCPacketScheduler::setFunctions13to20(uint16_t address, uint8_t functions)	//F20 F19 F18 F17 F16 F15 F14 F13
{
	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);
	uint8_t data[] = { B11011110, 0x00 }; 
	data[1] = functions;	//significant functions (F20--F13)
	p.addData(data, 2);
	p.setKind(function_packet_4_kind);
	p.setRepeat(FUNCTION_REPEAT);
	LokDataUpdate[LokStsgetSlot(address)].f2 = functions; //write into register to SAVE
	//return low_priority_queue.insertPacket(&p);
	return repeat_queue.insertPacket(&p);
}

bool DCCPacketScheduler::setFunctions21to28(uint16_t address, uint8_t functions)	//F28 F27 F26 F25 F24 F23 F22 F21
{
	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);
	uint8_t data[] = { B11011111, 0x00}; 
	data[1] = functions; //significant functions (F28--F21)
	p.addData(data, 2);
	p.setKind(function_packet_5_kind);
	p.setRepeat(FUNCTION_REPEAT);
	LokDataUpdate[LokStsgetSlot(address)].f3 = functions; //write into register to SAVE
	//return low_priority_queue.insertPacket(&p);
	return repeat_queue.insertPacket(&p);
}

byte DCCPacketScheduler::getFunktion0to4(uint16_t address)	//gibt Funktionszustand - F0 F4 F3 F2 F1 zurück
{
	return LokDataUpdate[LokStsgetSlot(address)].f0 & 0x1F;
}

byte DCCPacketScheduler::getFunktion5to8(uint16_t address)	//gibt Funktionszustand - F8 F7 F6 F5 zurück
{
	return LokDataUpdate[LokStsgetSlot(address)].f1 & 0x0F;
}

byte DCCPacketScheduler::getFunktion9to12(uint16_t address)	//gibt Funktionszustand - F12 F11 F10 F9 zurück
{
	return LokDataUpdate[LokStsgetSlot(address)].f1 >> 4;
}

byte DCCPacketScheduler::getFunktion13to20(uint16_t address)	//gibt Funktionszustand F20 - F13 zurück
{
	return LokDataUpdate[LokStsgetSlot(address)].f2;
}

byte DCCPacketScheduler::getFunktion21to28(uint16_t address)	//gibt Funktionszustand F28 - F21 zurück
{
	return LokDataUpdate[LokStsgetSlot(address)].f3;
}

//---------------------------------------------------------------------------------
//Special Function for programming, switch and estop:

bool DCCPacketScheduler::setBasicAccessoryPos(uint16_t address, bool state)
{
	return setBasicAccessoryPos(address, state, true);	//Ausgang aktivieren
}

bool DCCPacketScheduler::setBasicAccessoryPos(uint16_t address, bool state, bool activ)
{
	/*
	Accessory decoder packet format:
	================================
	1111..11 0 1000-0001 0 1111-1011 0 EEEE-EEEE 1
      Preamble | 10AA-AAAA | 1aaa-CDDX | Err.Det.B

      aaaAAAAAA -> 111000001 -> Acc. decoder number 1

	  UINT16 FAdr = (FAdr_MSB << 8) + FAdr_LSB;
	  UINT16 Dcc_Addr = FAdr >> 2	//aaaAAAAAA

	  Beispiel:
	  FAdr=0 ergibt DCC-Addr=0 Port=0;
	  FAdr=3 ergibt DCC-Addr=0 Port=3;
	  FAdr=4 ergibt DCC-Addr=1 Port=0; usw

      C on/off:    1 => on		// Ausgang aktivieren oder deaktivieren
      DD turnout: 01 => 2		// FAdr & 0x03  // Port
      X str/div:   1 => set to diverging  // Weiche nach links oder nach rechts 
		=> X=0 soll dabei Weiche auf Abzweig bzw. Signal auf Halt kennzeichnen.

     => COMMAND: SET TURNOUT NUMBER 2 DIVERGING

	 1111..11 0 1000-0001 0 1111-0011 0 EEEE-EEEE 1
	 => COMMAND: SET TURNOUT NUMBER 6 DIVERGING
	*/
	if (address > 0x7FF)	//check if Adr is ok, (max. 11-bit for Basic Adr)
		return false;

	DCCPacket p((address + TrntFormat) >> 2); //9-bit Address + Change Format Roco / Intellibox
	uint8_t data[1];
	data[0] = ((address + TrntFormat) & 0x03) << 1;	//0000-CDDX
	if (state == true)	//SET X Weiche nach links oder nach rechts 
		bitWrite(data[0], 0, 1);	//set turn
	if (activ == true )	//SET C Ausgang aktivieren oder deaktivieren 
		bitWrite(data[0], 3, 1);	//set ON

	p.addData(data, 1);
	p.setKind(basic_accessory_packet_kind);
	p.setRepeat(OTHER_REPEAT);

	if (notifyTrnt)
		notifyTrnt(address, state, activ);

	bitWrite(BasicAccessory[address / 8], address % 8, state);	//pro SLOT immer 8 Zustände speichern!

	//return high_priority_queue.insertPacket(&p);
	return e_stop_queue.insertPacket(&p);
}

bool DCCPacketScheduler::getBasicAccessoryInfo(uint16_t address)
{
 	switch (TrntFormat) {
		case IB: address = address + IB; break;
	}

	return bitRead(BasicAccessory[address / 8], address % 8);	//Zustand aus Slot lesen
}

bool DCCPacketScheduler::opsProgDirectCV(uint16_t CV, uint8_t CV_data)
{
	//for CV#1 is the Adress 0
	//Long-preamble   0  0111CCAA  0  AAAAAAAA  0  DDDDDDDD  0  EEEEEEEE  1 
	//CC=10 Bit Manipulation
	//CC=01 Verify byte
	//CC=11 Write byte 
	
	//check if CV# is between 0 - 1023
	if (CV > 1023) {
		if (notifyCVNack)
			notifyCVNack();
		return false;
	}
	
	DCCPacket p(((CV >> 8) & B11) | B01111100);
	uint8_t data[] = { 0x00 , 0x00};
	data[0] = CV & 0xFF;
	data[1] = CV_data;
	p.addData(data, 2);
	p.setKind(ops_mode_programming_kind);	//always use short Adress Mode!
	p.setRepeat(ProgRepeat);

	if (railpower != SERVICE)	//time to wait for the relais!
		opsDecoderReset(RSTsRepeat);	//send first a Reset Packet
	setpower(SERVICE, true);
	current_cv_bit = 0xFF; //write the byte!
	current_ack_read = false;
	ack_monitor_time = micros();
	current_cv = CV;
	current_cv_value = CV_data;

	//-------opsDecoderReset(RSTsRepeat);	//send first a Reset Packet
	ops_programmming_queue.insertPacket(&p);
	//-----opsDecoderReset(RSTcRepeat);	//send a Reset while waiting to finish
	return opsVerifyDirectCV(current_cv,current_cv_value); //verify bit read
}

bool DCCPacketScheduler::opsVerifyDirectCV(uint16_t CV, uint8_t CV_data)
{
	//for CV#1 is the Adress 0
	//Long-preamble   0  0111CCAA  0  AAAAAAAA  0  DDDDDDDD  0  EEEEEEEE  1 
	//CC=10 Bit Manipulation
	//CC=01 Verify byte
	//CC=11 Write byte 
	
	//check if CV# is between 0 - 1023
	if (CV > 1023) {
		if (notifyCVNack)
			notifyCVNack();
		return false;
	}
	
	DCCPacket p(((CV >> 8) & B11) | B01110100);
	uint8_t data[2];
	data[0]	= CV & 0xFF;
	data[1] = CV_data;
	p.addData(data, 2);
	p.setKind(ops_mode_programming_kind);	//always use short Adress Mode!
	p.setRepeat(ProgRepeat);

	if (railpower != SERVICE)	//time to wait for the relais!
		opsDecoderReset(RSTsRepeat);	//send first a Reset Packet
	setpower(SERVICE, true);
	current_cv_bit = 0xF0; //verify the byte!
	current_ack_read = false;
	ack_monitor_time = micros();
	current_cv = CV;
	current_cv_value = CV_data;

	opsDecoderReset(RSTsRepeat);	//send first a Reset Packet
	return ops_programmming_queue.insertPacket(&p);
	//------opsDecoderReset(RSTcRepeat);	//send a Reset while waiting to finish
	//return opsDecoderReset(RSTcRepeat);	//send a Reset while waiting to finish
}

bool DCCPacketScheduler::opsReadDirectCV(uint16_t CV, uint8_t bitToRead, bool bitSet)
{
	//for CV#1 is the Adress 0
	//long-preamble   0  011110AA  0  AAAAAAAA  0  111KDBBB  0  EEEEEEEE  1 
	//Bit Manipulation
	//BBB represents the bit position 
	//D contains the value of the bit to be verified or written
	//K=1 signifies a "Write Bit" operation and K=0 signifies a "Bit Verify" 
	
	//check if CV# is between 0 - 1023
	if (CV > 1023) {
		if (notifyCVNack)
			notifyCVNack();
		return false;
	}
	
	if (current_cv_bit > 7 || bitToRead == 0) {
		current_cv_bit = 0;
		bitToRead = 0;
	}
	
	if (railpower != SERVICE)	//time to wait for the relais!
		opsDecoderReset(RSTsRepeat);	//send first a Reset Packet
	setpower(SERVICE, true);
	current_ack_read = false;
	ack_monitor_time = micros();
	current_cv = CV;
	
	DCCPacket p(((CV >> 8) & B11) | B01111000);
	uint8_t data[] = { 0x00 , 0x00};
	data[0] = CV & 0xFF;
	data[1] = B11100000 | (bitToRead & 0x07) | (bitSet << 3);	//verify Bit is "bitSet"? ("1" or "0")
	p.addData(data, 2);
	p.setKind(ops_mode_programming_kind);	//always use short Adress Mode!
	p.setRepeat(ProgRepeat);
	
	if (bitToRead == 0) {
		opsDecoderReset(RSTsRepeat);	//send first a Reset Packet
	}
	
	return ops_programmming_queue.insertPacket(&p);
	//--------opsDecoderReset(RSTcRepeat);	//send a Reset while waiting to finish
	//return opsDecoderReset(RSTcRepeat);	//send a Reset while waiting to finish
}



bool DCCPacketScheduler::opsProgramCV(uint16_t address, uint16_t CV, uint8_t CV_data)
{
	//format of packet:
	// {preamble} 0 [ AAAAAAAA ] 0 111011VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (write)
	// {preamble} 0 [ AAAAAAAA ] 0 111001VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (verify/read)
	// {preamble} 0 [ AAAAAAAA ] 0 111010VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (bit manipulation)
	// only concerned with "write" form here!!!

	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);
	uint8_t data[] = { 0x00, 0x00, 0x00 };

	// split the CV address up among data uint8_ts 0 and 1
	data[0] = ((CV >> 8) & B11) | B11101100;
	data[1] = CV & 0xFF;
	data[2] = CV_data;

	p.addData(data, 3);
	p.setKind(pom_mode_programming_kind);
	p.setRepeat(ProgRepeat);

	//return low_priority_queue.insertPacket(&p);	//Standard

	//opsDecoderReset();	//send first a Reset Packet
	return ops_programmming_queue.insertPacket(&p);
	//return e_stop_queue.insertPacket(&p);
}

bool DCCPacketScheduler::opsPOMwriteBit(uint16_t address, uint16_t CV, uint8_t Bit_data)
{
	//format of packet:
	// {preamble} 0 [ AAAAAAAA ] 0 111011VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (write)
	// {preamble} 0 [ AAAAAAAA ] 0 111001VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (verify/read)
	// {preamble} 0 [ AAAAAAAA ] 0 111010VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (bit manipulation)
	// only concerned with "write" form here!!!

	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);
	uint8_t data[] = { 0x00, 0x00, 0x00 };

	// split the CV address up among data uint8_ts 0 and 1
	data[0] = ((CV >> 8) & B11) | B11101000;
	data[1] = CV & 0xFF;
	data[2] = Bit_data & 0x0F;

	p.addData(data, 3);
	p.setKind(pom_mode_programming_kind);
	p.setRepeat(ProgRepeat);

	//return low_priority_queue.insertPacket(&p);	//Standard

	//opsDecoderReset();	//send first a Reset Packet
	return ops_programmming_queue.insertPacket(&p);
	//return e_stop_queue.insertPacket(&p);
}

bool DCCPacketScheduler::opsPOMreadCV(uint16_t address, uint16_t CV)
{
	//format of packet:
	// {preamble} 0 [ AAAAAAAA ] 0 111001VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (verify/read)

	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);
	uint8_t data[] = { 0x00, 0x00, 0x00 };

	// split the CV address up among data uint8_ts 0 and 1
	data[0] = ((CV >> 8) & B11) | B11100100;
	data[1] = CV & 0xFF;
	data[2] = 0;

	p.addData(data, 3);
	p.setKind(pom_mode_programming_kind);
	p.setRepeat(ProgRepeat);
	
	#if defined(GLOBALRAILCOMREADER)
	POMCVAdr = CV;
	#endif

	return ops_programmming_queue.insertPacket(&p);
	//return e_stop_queue.insertPacket(&p);
}

//broadcast Decoder ResetPacket
bool DCCPacketScheduler::opsDecoderReset(uint8_t repeat)
{
	// {preamble} 0 00000000 0	00000000 0 EEEEEEEE	1
	DCCPacket p(0);	//Adr = 0
	uint8_t data[] = { 0x00 };
	p.addData(data, 1);
	p.setKind(ops_mode_programming_kind); //always use short Adress Mode!
	p.setRepeat(repeat);
	return ops_programmming_queue.insertPacket(&p);
}
    
//broadcast e-stop command
void DCCPacketScheduler::eStop(void)
{
    // 111111111111 0 00000000 0 01DC0001 0 EEEEEEEE 1
	// C = by default contain one additional speed bit
	// D = direction ("1" the locomotive should	move in	the	forward	direction)
    DCCPacket e_stop_packet(0); //address 0
    uint8_t data[] = {0x61}; //01100001
    e_stop_packet.addData(data,1);
    e_stop_packet.setKind(e_stop_packet_kind);
	e_stop_packet.setRepeat(E_STOP_REPEAT);
    e_stop_queue.insertPacket(&e_stop_packet);

	setpower(ESTOP, true);

    return;
}

/*
bool DCCPacketScheduler::eStop(uint16_t address)
{
    // 111111111111 0	0AAAAAAA 0 01001001 0 EEEEEEEE 1
    // or
    // 111111111111 0	0AAAAAAA 0 01000001 0 EEEEEEEE 1
    DCCPacket e_stop_packet(address);
    uint8_t data[] = {0x41}; //01000001
    e_stop_packet.addData(data,1);
//    e_stop_packet.setKind(e_stop_packet_kind);
	e_stop_packet.setRepeat(E_STOP_REPEAT);
//    e_stop_queue.insertPacket(&e_stop_packet);
    //now, clear this packet's address from all other queues
    //high_priority_queue.forget(address);	//no locos
	//low_priority_queue.forget(address);
	//repeat_queue.forget(address);
	//periodic_refresh_queue.forget(address);			
	e_stop_packet.setKind(speed_packet_kind);
	repeat_queue.insertPacket(&e_stop_packet);
}
*/


//for CV read, to detect ACK
void DCCPacketScheduler::setCurrentLoadPin(uint8_t pin) {
	current_load_pin = pin;
	pinMode(pin, INPUT);
}

//to be called periodically within loop()
//checks queues, puts whatever's pending on the rails via global current_packet
void DCCPacketScheduler::update(void) {
	//CV read on Prog.Track:
	if (current_packet_service == true && current_ack_read == false && (micros() - ack_monitor_time >= ACK_TIME_WAIT_TO_MONITOR) ) {
		uint16_t current_load_now = analogRead(current_load_pin);	//get current value
		/*
		Serial.print(current_load_now);
		Serial.print(",");
		*/	
		//was there a ACK for this packet?	
		if (current_load_now > ACK_SENCE_VALUE) {	//AREF 1.1 Volt = 200 | AREF 5.0 Volt = 15
			if (ack_received_now == false) {
				ack_received_now = true; //we are inside a ACK
				ack_received_time = micros();
				
				//Serial.println();
			}
			
		}
		else {	
			if (ack_received_now == true) {
				ack_received_now = false;
				if (micros() - ack_received_time >= ACK_SENCE_TIME) {		//length of ack received?
					/*
					Serial.print(micros() - ack_monitor_time);
					Serial.println("*");
					Serial.print(micros() - ack_received_time);
					Serial.print("-b");
					Serial.println(current_cv_bit);
					*/	
					current_ack_read = true;
					if (current_cv_bit <= 7)  //CV read....?
						bitWrite(current_cv_value,current_cv_bit,1);	//ACK, so bit is 'one'!
					else {	//return cv value:
						if (notifyCVVerify)		//Verify the Value to device!
							notifyCVVerify(current_cv,current_cv_value);
						
						//No Power ON!
						setpower(ON, true);		//need to switch to accept more programming information
					}
				}
			}
		}
		//ENDE mesure currend load for CV# read
	}
	
	//Get next packet:
	if (get_next_packet) //if the ISR needs a packet:
	{
		DCCPacket p;

		if (ops_programmming_queue.notEmpty()) {	//first Check if ops Service Mode (programming)?
			ops_programmming_queue.readPacket(&p);
		}
		else {

			if (railpower == SERVICE) {		//if command station was in ops Service Mode, switch power off!
				if (current_ack_read == false && current_cv_bit > 7) {	//No ACK for the Data!!!
					if (current_cv_value > 0) { //read only again if there is any response
						/*
						Serial.println("wrong!");
						*/
						opsDecoderReset(RSTsRepeat);	//send first a Reset Packet
						ops_programmming_queue.readPacket(&p);
						
						opsReadDirectCV(current_cv, 0); //read again!!
					}
					else {	//Return no ACK while programming
						if (notifyCVNack)
							notifyCVNack();
						
						setpower(ON, true);	
					}	
				}
			}
			if (current_cv_bit <= 7 && !ops_programmming_queue.notEmpty())	{ //CV read: more bit to read...?
				if (current_ack_read == false) 	//no ACK - the bit is zero!
					bitWrite(current_cv_value,current_cv_bit,0);
				/*
				Serial.print(current_cv_bit);					
				Serial.print("-");	
				Serial.println(current_ack_read);
				*/				
				current_cv_bit++;	//get next bit

				if (current_cv_bit > 7) {  //READY: read all 8 bit of the CV value:
					current_cv_bit = 0xFF;	//STOP here and reset to default
					//check if value is correct?
					opsDecoderReset(RSTsRepeat);	//send first a Reset Packet
					ops_programmming_queue.readPacket(&p);
					opsVerifyDirectCV(current_cv,current_cv_value); //verify bit read
					/*
					Serial.print("read:");
					Serial.print(current_cv);
					Serial.print(" - ");
					Serial.println(current_cv_value);
					*/
				}
				else {	//read next bit:
				
					opsReadDirectCV(current_cv, current_cv_bit);	//ask for the next bit!
					ops_programmming_queue.readPacket(&p);

				}
			}
			else {
				
				if (e_stop_queue.notEmpty() && (packet_counter % ONCE_REFRESH_INTERVAL)) {	//if there's an e_stop packet, send it now!
					e_stop_queue.readPacket(&p); //nothing more to do. e_stop_queue is a repeat_queue, so automatically repeats where necessary.
				}
				else {
					if (repeat_queue.notEmpty()) {	//for each packet to send it fast, before it stay long waiting in periodic!
						repeat_queue.readPacket(&p);
						periodic_refresh_queue.insertPacket(&p);
					}
					else {
						//	if (periodic_refresh_queue.notEmpty()) // && periodic_refresh_queue.notRepeat(last_packet_address))
						if (railpower != ESTOP)
							periodic_refresh_queue.readPacket(&p);
					}
					/*if (p.getKind() == speed_packet_kind && railpower == ESTOP) {
						e_stop_queue.readPacket(&p);
					}*/
				}
			}	//read CV by bit
		}
		++packet_counter;	//to not repeat only one queue!
		//last_packet_address = p.getAddress(); //remember the address to compare with the next packet
		current_packet_size = p.getBitstream(current_packet); //feed to the starting ISR.
		
		/*
		if (p.getAddress() != 255) {	
			Serial.print(p.getAddress());		
			Serial.print("-");
			for (byte v = 0; v < current_packet_size; v++) {
				for (byte c = 0; c < 8; c++)
					Serial.print(bitRead(current_packet[v],7-c), BIN);
				Serial.print(" ");
			}
			Serial.println(current_packet_size);
		}
		*/
		
		if (railpower == SERVICE) {
			current_packet_service = true;
		}
		else {
			current_packet_service = false;
		}
		
		//#if defined(ESP32)
		//portENTER_CRITICAL_ISR(&timerMux);
		//#endif
		//Ready: Next Packet for the ISR!
		get_next_packet = false;
		//#if defined(ESP32)
		//portEXIT_CRITICAL_ISR(&timerMux);
		//#endif
	}
}

/*
-> old function:
void DCCPacketScheduler::update(void) 
{
  DCC_waveform_generation_hasshin();

  //TODO ADD POM QUEUE?
  if(!current_uint8_t_counter) //if the ISR needs a packet:
  {
    DCCPacket p;
    //Take from e_stop queue first, then high priority queue.
    //every fifth packet will come from low priority queue.
    //every 20th packet will come from periodic refresh queue. (Why 20? because. TODO reasoning)
    //if there's a packet ready, and the counter is not divisible by 5
    //first, we need to know which queues have packets ready, and the state of the this->packet_counter.
    if( !e_stop_queue.isEmpty() ) //if there's an e_stop packet, send it now!
    {
      //e_stop
      e_stop_queue.readPacket(&p); //nothing more to do. e_stop_queue is a repeat_queue, so automatically repeats where necessary.
    }
    else
    {
      bool doHigh = high_priority_queue.notEmpty() && high_priority_queue.notRepeat(last_packet_address);
      bool doLow = low_priority_queue.notEmpty() && low_priority_queue.notRepeat(last_packet_address) &&
                  !((packet_counter % LOW_PRIORITY_INTERVAL) && doHigh);
      bool doRepeat = repeat_queue.notEmpty() && repeat_queue.notRepeat(last_packet_address) &&
                  !((packet_counter % REPEAT_INTERVAL) && (doHigh || doLow));
//NEW:
      bool doRefresh = periodic_refresh_queue.notEmpty() && periodic_refresh_queue.notRepeat(last_packet_address) &&
                  !((packet_counter % PERIODIC_REFRESH_INTERVAL) && (doHigh || doLow || doRepeat));
      //examine queues in order from lowest priority to highest.
      if(doRefresh)
      {
        periodic_refresh_queue.readPacket(&p);
        ++packet_counter;
      }
      else if(doRepeat)
//NEW (END)
      //if(doRepeat)
      {
        //Serial.println("repeat");
        repeat_queue.readPacket(&p);
        ++packet_counter;
      }
      else if(doLow)
      {
        //Serial.println("low");
        low_priority_queue.readPacket(&p);
        ++packet_counter;
      }
      else if(doHigh)
      {
        //Serial.println("high");
        high_priority_queue.readPacket(&p);
        ++packet_counter;
      }
      //if none of these conditions hold, DCCPackets initialize to the idle packet, so that's what'll get sent.
      //++packet_counter; //it's a uint8_t; let it overflow, that's OK.
      //enqueue the packet for repitition, if necessary:
      //Serial.println("idle");
 //repeatPacket(&p);	//bring the packet into the repeat queue
    }
    last_packet_address = p.getAddress(); //remember the address to compare with the next packet
    current_packet_size = p.getBitstream(current_packet); //feed to the starving ISR.
    //output the packet, for checking:
    //if(current_packet[0] != 0xFF) //if not idle
    //{
    //  for(uint8_t i = 0; i < current_packet_size; ++i)
    //  {
    //    Serial.print(current_packet[i],BIN);
    //    Serial.print(" ");
    //  }
    //  Serial.println("");
    //}
    current_uint8_t_counter = current_packet_size;
  }
}
*/
/*
//--------------------------------------------------------------------------------------------
//Gibt aktuellen Lokstatus an Anfragenden zurück
void DCCPacketScheduler::getLocoStateFull(uint16_t adr)
{
	byte Slot = LokStsgetSlot(adr);
	byte Speed = LokDataUpdate[Slot].speed;
	if (notifyLokAll)
		notifyLokAll(adr, LokDataUpdate[Slot].adr >> 14, Speed, LokDataUpdate[Slot].f0 & 0x1F, 
		LokDataUpdate[Slot].f1, LokDataUpdate[Slot].f2, LokDataUpdate[Slot].f3);
}
*/

//--------------------------------------------------------------------------------------------
//aktuellen Zustand aller Funktionen und Speed der Lok
void DCCPacketScheduler::getLocoData(uint16_t adr, uint8_t data[])
{
	//uint8_t Steps, uint8_t Speed, uint8_t F0, uint8_t F1, uint8_t F2, uint8_t F3
	byte Slot = LokStsgetSlot(adr);
	data[0] = LokDataUpdate[Slot].adr >> 14; 	//Steps
	data[1] = LokDataUpdate[Slot].speed;
	data[2] = LokDataUpdate[Slot].f0 & 0x1F;	//F0 - F4
	data[3] = LokDataUpdate[Slot].f1;
	data[4] = LokDataUpdate[Slot].f2;
	data[5] = LokDataUpdate[Slot].f3;
}

//--------------------------------------------------------------------------------------------
//Gibt aktuelle Fahrtrichtung der Angefragen Lok zurück
byte DCCPacketScheduler::getLocoDir(uint16_t adr)
{
	return bitRead(LokDataUpdate[LokStsgetSlot(adr)].speed, 7);
}
//--------------------------------------------------------------------------------------------
//Gibt aktuelle Geschwindigkeit der Angefragen Lok zurück
byte DCCPacketScheduler::getLocoSpeed(uint16_t adr)
{
	return LokDataUpdate[LokStsgetSlot(adr)].speed & 0x7F;
}

//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
byte DCCPacketScheduler::LokStsgetSlot(uint16_t adr)		//gibt Slot für Adresse zurück / erzeugt neuen Slot (0..126)
{
	byte Slot;
	for (Slot = 0; Slot < SlotMax; Slot++) {
		if ((LokDataUpdate[Slot].adr & 0x3FFF) == adr)
			return Slot;	//Lok gefunden, Slot ausgeben
		if ((LokDataUpdate[Slot].adr & 0x3FFF) == 0) {
			//Empty? neuer freier Slot - keine weitern Lok's!
			LokStsSetNew(Slot, adr);	//Eintragen
			return Slot;
		}
	}
	//kein Slot mehr vorhanden!
	//start am Anfang mit dem Überschreiben vorhandender Slots
	Slot = slotFullNext;
	LokStsSetNew(Slot, adr);	//clear Slot!
	slotFullNext++;
	if (slotFullNext >= SlotMax)
		slotFullNext = 0;
	return Slot;
}

//--------------------------------------------------------------------------------------------
void DCCPacketScheduler::LokStsSetNew(byte Slot, uint16_t adr)	//Neue Lok eintragen mit Adresse
{
	LokDataUpdate[Slot].adr = adr | (DCCdefaultSteps << 14); //0x4000; //0xC000;	// c = '3' => 128 Fahrstufen
	LokDataUpdate[Slot].speed = 0x80;	//default direction is forward
	LokDataUpdate[Slot].f0 = 0x00;
	LokDataUpdate[Slot].f1 = 0x00;
	LokDataUpdate[Slot].f2 = 0x00;
	LokDataUpdate[Slot].f3 = 0x00;
	
	//generate first drive information:
	#if defined(InitLocoDirect)
	setSpeed(LokDataUpdate[Slot].adr, LokDataUpdate[Slot].speed); 
	#endif
}

/*
// --------------------------------------------------------------------------------------------
bool DCCPacketScheduler::LokStsIsEmpty(byte Slot)	//prüft ob Datenpacket/Slot leer ist?
{
if ((LokDataUpdate[Slot].adr & 0x3FFF) == 0x0000)
return true;
return false;
}

//--------------------------------------------------------------------------------------------
uint16_t DCCPacketScheduler::LokStsgetAdr(byte Slot)			//gibt Lokadresse des Slot zurück, wenn 0x0000 dann keine Lok vorhanden
{
//	if (!LokDataUpdateIsEmpty(Slot))
		return LokDataUpdate[Slot].adr & 0x3FFF;	//Addresse zurückgeben
//	return 0x0000;
}

//--------------------------------------------------------------------------------------------
byte DCCPacketScheduler::getNextSlot(byte Slot)	//gibt nächsten genutzten Slot
{
	byte nextS = Slot;
	for (byte i = 0; i < SlotMax; i++) {
		nextS++;	//nächste Lok
		if (nextS >= SlotMax)
			nextS = 0;	//Beginne von vorne
		if (LokStsIsEmpty(nextS) == false)
			return nextS;
	}
	return nextS;
}

//--------------------------------------------------------------------------------------------
void DCCPacketScheduler::setFree(uint16_t adr)		//Lok aus Slot nehmen
{
	byte Slot = LokStsgetSlot(adr);
	LokDataUpdate[Slot].adr = 0x0000;
	LokDataUpdate[Slot].speed = 0x00;
	LokDataUpdate[Slot].f0 = 0x00;
	LokDataUpdate[Slot].f1 = 0x00;
	LokDataUpdate[Slot].f2 = 0x00;
	LokDataUpdate[Slot].f3 = 0x00;
}
*/

//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
#if defined(GLOBALRAILCOMREADER)
//Interrupt routine for reading via Serial
ISR(USART3_RX_vect) {
	DCCPacketScheduler::handle_RX_interrupt();	 //weiterreichen an die Funktion
}

//--------------------------------------------------------------------------------------------
// Interrupt handling for receive Data
// static 
inline void DCCPacketScheduler::handle_RX_interrupt()
{
  if (active_object)
  {
	active_object->RailComReceive();	//Byte lesen
  }
}

//--------------------------------------------------------------------------------------------
//Reading RailCom Data:
void DCCPacketScheduler::RailComReceive(void) {
	uint8_t c = RailComDecodeInData(UDR3);
	if (RailComID > 0x3F) {
		if ((c >> 2) < 3) //app:pom, app:adr_low, app:adr_high -> read only 12 Bit!
			RailComID = c;
	}
	else if (RailComData == 0xFF)
		RailComData = c;
	/*
	if (c < 0x40) {    //gültige Railcom Message empfangen
		//Serial.print(c, BIN);
		//Serial.print(":");
	}
	*/
}

//Decoding RailCom Received Data
uint8_t DCCPacketScheduler::RailComDecodeInData(uint8_t in) {
  switch (in) {
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
#endif