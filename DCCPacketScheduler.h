/*
 * DCC Waveform Generator v6.0.1
 *
 * Author: Philipp Gahtow digitalmoba@arcor.de
 *		   Don Goodman-Wilson dgoodman@artificial-science.org
 *
 * based on software by Wolfgang Kufer, http://opendcc.de
 *
 * modified by Philipp Gahtow
 * Copyright 2022 digitalmoba@arcor.de, http://pgahtow.de
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
 *  
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *  
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Hardware requirements:
 *     * A DCC booster with Data and GND wired to pin configured in setup routine (PIN 6) - BASIC OUTPUT!
 *     * A locomotive/switch with a DCC decoder installed (NMRA).
 * 
 * Attention: only can handel CV Direct Bit/Byte Mode
 *
 * change log: 
 * - add a store for active loco, so you can request the actual state
 * - add a store for BasicAccessory states
 * - add a repeat queue for Speed and Function packets
 * - add Function support F13-F20 and F21-F28
 * - add POM CV Programming
 * - add BasicAccessory increment 4x (Intellibox - ROCO)
 * - add request for state of Loco funktion F0 - F28
 * - support DCC generation with Timer1 or Timer2
 * - add notify of BasicAccessory even when power is off
 * - change praeambel to 16 Bit for Railcom support
 * - add Railcom hardware support and inverted DCC signal
 * - add Rail Power management
 * - add CV read
 * - add track load information
 * - optimize RailCom CutOut
 * - fix RailCom de/aktivation
 * - add variable packet repeat for programming and reset
 * - fix Power notify for Service Mode
 * - add support for Arduino DUE
 * - fix DCC speed steps
 * - add long Adress Mode for POM
 * - fix getBasicAccessoryInfo for Adress zero
 * - add request array for loco information
 * - add support for Arduino ESP8266
 * - add additional DCC output signal -without- "power off" and RailCom! (DCCHardware.c)
 * - fix idle_packet_kind to send a correct idll packet 0xFF 0x00 0xFF (inside DCCPacket.cpp and DCCHardware.c)
 * - fix accessory packet address problem that filters out packets (DCCPAcketQueue.cpp)
 * - removed unused variable with the name "last_packet_address"
 * - change, new loco with speed direction forward (0x80)
 * - change loco slot direction bit now under speed
 * - change start up power status to ON!
 * - when loco request the first time (new loco), start also to send drive information directly
		- activate by config #define InitLocoDirect
 * - add truth table inside DCCHardware.c for easy adjust the output states! 
 * - add support for ESP32 and adjust ESP8266
 * - add support for AREF with 1.1 Volt and adjust the CV# read parts.
 * - add a DCC timer interrupt start/stop function because of problems crashing ESP32 and ESP8266 when EEPROM.commit runs
 * - add EEPROM.commit statement for ESP32 and ESP8266
 * - optimize CV# read with new detection time
 * - add NVS to store EEPROM data on ESP32
 * - add active label to notifyTrnt message
 * - add CV to notifyCVNack return
 * - fix reset start packet count
 * - fix direct programming handle
 * - add request for actual RailCom cutout status
 * - fix ACK Value for ESP8266
 * - fix DCC timing on ESP32
 * - change ACK detection and add state machine for CV direct	
 * - add function bit control from F29...F32767
 * - add function setExtAccessoryPos
 */

#ifndef __DCCCOMMANDSTATION_H__
#define __DCCCOMMANDSTATION_H__
#include "DCCPacket.h"
#include "DCCPacketQueue.h"

/*******************************************************************/
//#define PROG_DEBUG	//Serial output of Prog Informaton

#if defined(ESP8266) //ESP8266 or WeMos D1 mini
#define ACK_SENCE_VALUE 10	//WeMos has a voltage divider for 3.1 Volt -> we not want to modify the board!
#define	ACK_SENCE_DIFF 0	//Differenz beim Konstanten Strom am Gleis (Abweichung +/-)
#define ACK_SENCE_TIME 2	//Dauer bis ein ACK erkannt wird
#else
#define ACK_SENCE_VALUE 90	//Value = 200 for use with AREF = 1.1 Volt analog Refence Voltage; (Value = 15 for AREF = 5.0 Volt)
#define	ACK_SENCE_DIFF 33	//Differenz beim Konstanten Strom am Gleis (Abweichung +/-)
#define ACK_SENCE_TIME 8	//Dauer bis ein ACK erkannt wird
#endif

//read value again if verify fails:
#define CV_BIT_MAX_TRY_READ 	4	//times to try in Bit-Mode
#define CV_BYTE_MAX_TRY_READ	1	//times to try in Byte-Mode

/*******************************************************************/
//When loco request the first time (new loco), start also to send drive information directly
//#define InitLocoDirect

//Function F29-F68 control
#define EXTENDFUNCTION		//activate functions over F28

/*******************************************************************/
//Protokoll can handel max 16384 switch (Weichenzustände max 16384):
#if defined(__SAM3X8E__)
// Arduino Due Board follows
#define AccessoryMax 4096	//max DCC 2048 Weichen / 8 = 255 byte 
#define SlotMax 255			//Slots für Lokdaten
#define PERIODIC_REFRESH_QUEUE_SIZE 255

#elif defined(ESP8266) || defined(ESP32) //ESP8266 or WeMos D1 mini
// Arduino ESP8266 Board follows
#define AccessoryMax 4096	//max DCC 2048 Weichen / 8 = 255 byte 
#define SlotMax 255			//Slots für Lokdaten
#define PERIODIC_REFRESH_QUEUE_SIZE 255

#elif defined(__AVR_ATmega1284P__) 
//more then 8 KB RAM
#define AccessoryMax 2048	//max DCC 2048 Weichen / 8 = 255 byte 
#define SlotMax 255			//Slots für Lokdaten
#define PERIODIC_REFRESH_QUEUE_SIZE 200

#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) 
//8 KB RAM
#define AccessoryMax 1024	//max DCC 2048 Weichen / 8 = 255 byte 
#define SlotMax 80			//Slots für Lokdaten
#define PERIODIC_REFRESH_QUEUE_SIZE 100
//#define GLOBALRAILCOMREADER	//Activate Global RailCom Reading in CutOut over Serial3 - now in external Code???

#elif defined (__AVR_ATmega644P__)	
//4 KB RAM
#define AccessoryMax 512	//normal 512 Weichen / 8 = 64 byte
#define SlotMax 36			//Slots für Lokdaten
#define PERIODIC_REFRESH_QUEUE_SIZE 70

#else	
//less then 2,5 KB RAM
#define AccessoryMax 128		//64 Weichen / 8 = 8 byte
#define SlotMax 15			//Slots für Lokdaten
#define PERIODIC_REFRESH_QUEUE_SIZE 60
#endif
/*******************************************************************/

#define E_STOP_QUEUE_SIZE        15
#define REPEAT_QUEUE_SIZE        25
#define PROG_QUEUE_SIZE			 5

//How often a packet is repeat:
#define ONCE_REFRESH_INTERVAL	4	//send estop, switch, pom each "second" packet

//Repaet for Packetkinds:
#define SPEED_REPEAT      3
#define FUNCTION_REPEAT   3
#define E_STOP_REPEAT     6
#define RESET_START_REPEAT	  25	//(default, read fom EEPROM)
#define RESET_CONT_REPEAT	  6		//(default, read fom EEPROM)
#if defined(ESP8266) //ESP8266 or WeMos D1 mini
#define OPS_MODE_PROGRAMMING_REPEAT 18	//(default, read fom EEPROM)
#else
#define OPS_MODE_PROGRAMMING_REPEAT 7	//(default, read fom EEPROM)
#endif
#define OTHER_REPEAT      9		//for example accessory paket

//State of Railpower:
#define OFF	0x02		//no power on the rails
#define ON	0x00		//signal on the rails
#define ESTOP 0x01		//no Loco drive but rails have power
#define SHORT 0x04
#define SERVICE 0x08	//system is in CV programming mode

//Trnt message paket format (inc)
#define ROCO 0
#define IB 4

//DCC Speed Steps
#define DCC14	0x01
#define DCC28	0x02
#define DCC128	0x03

//EEPROM Configuration Store:
#define EEPROMRailCom 50
#define EEPROMProgReadMode 53	//Auslese-Modus: 0=Nichts, 1=Bit, 2=Byte, 3=Beides
#define EEPROMRSTsRepeat 60		//RESET_START_REPEAT
#define EEPROMRSTcRepeat 61		//RESET_CONT_REPEAT
#define EEPROMProgRepeat 62		//PROGRAMMING_REPEAT

//PROG STATUS States 'ProgState':
#define ProgStart		0x00
#define ProgACKRead		0x01
#define ProgBitRead		0x02	//Bit read
#define ProgVerifyCV	0x04	//Byte read
#define ProgWriteByte	0x05	//Byte write
#define ProgSuccess		0x0F
#define ProgFail		0xF0
#define ProgEnde		0xFF
		
//PROG Mode Select
#define ProgModeBit			0x10
#define ProgModeBitVerify	0x11
#define ProgModeByte		0x20
#define ProgModeByteVerify  0x21
#define ProgModeWriteByte	0x30

typedef struct	//Lokdaten	(Lok Events)
{
	uint16_t adr;		// SS1, SS0, A13, A12| A11, A10, A9, A8| A7, A6, A5, A4| A3, A2, A1, A0
	// A0-A13 = Adresse
	// SS = Fahrstufen-speedsteps (0=error, 1=14, 2=28, 3=128) 
	uint8_t speed;	//Dir, Speed 0..127 (0x00 - 0x7F) -> 0SSS SSSS + (0x80) -> D000 0000
	uint8_t f0;		//X   X   X   F0 | F4  F3  F2  F1			
	uint8_t f1;		//F12 F11 F10 F9 | F8  F7  F6  F5	
	uint8_t f2;		//F20 F19 F18 F17| F16 F15 F14 F13
	uint8_t f3;		//F28 F27 F26 F25| F24 F23 F22 F21 
} NetLok;

class DCCPacketScheduler
{
  public:
  
    DCCPacketScheduler(void);
    
    //for configuration
    //void setDefaultSpeedSteps(uint8_t new_speed_steps);
	void setup(uint8_t pin, uint8_t pin2, uint8_t steps = DCC128, uint8_t format = ROCO, uint8_t power = ON); 	//for any post-constructor initialization - with RailCom
	void enable_additional_DCC_output(uint8_t pin);	//extra DCC signal for S88/LocoNet without Shutdown and Railcom
	void disable_additional_DCC_output(void);
	
	void loadEEPROMconfig(void);	//Load Configuration from EEPROM

	//more specific functions:
	void setpower(uint8_t state, bool notify = false);		//set Mode of output aktive/inactive - should notify other clients?
    byte getpower(void);		//return the Mode of output power
	void setrailcom(bool rc = true);	//to de-/activate RailCom output
	bool getrailcom(void);		//return the State of RailCom output
	void eStop(void); //Broadcast Stop Packet For All Decoders
	// bool eStop(uint16_t address); //just one specific loco -> use setSpeed with speed = 1
    
    //for enqueueing packets
    bool setSpeed(uint16_t address, uint8_t speed); //use default speed steps!
    bool setSpeed14(uint16_t address, uint8_t speed); //new_speed: [-13,13]
    bool setSpeed28(uint16_t address, uint8_t speed); //new_speed: [-28,28]
    bool setSpeed128(uint16_t address, uint8_t speed); //new_speed: [-127,127]

	//void getLocoStateFull(uint16_t adr);	//aktuellen Zustand aller Funktionen und Speed der Lok
	void getLocoData(uint16_t adr, uint8_t data[]);	//aktuellen Zustand aller Funktionen und Speed der Lok
	byte getLocoDir(uint16_t adr); //Gibt aktuelle Fahrtrichtung der angefragen Lok zurück
	byte getLocoSpeed(uint16_t adr);	//Gibt aktuelle Geschwindigkeit der angefragten Lok zurück
    
    //the function methods are NOT stateful; you must specify all functions each time you call one
    //keeping track of function state is the responsibility of the calling program.
	void setLocoFunc(uint16_t address, uint8_t type, uint8_t fkt);	//type => 0 = AUS; 1 = EIN; 2 = UM; 3 = error | fkt => 0...68
	void setLocoFuncBinary(uint16_t address, uint8_t low, uint8_t high);	//Binärzustandsadressen von 29 bis 32767
    bool setFunctions0to4(uint16_t address, uint8_t functions);	//- F0 F4 F3 F2 F1
    bool setFunctions5to8(uint16_t address, uint8_t functions);	//- F8 F7 F6 F5
    bool setFunctions9to12(uint16_t address, uint8_t functions);	//- F12 F11 F10 F9
	bool setFunctions13to20(uint16_t address, uint8_t functions);	//F20 F19 F18 F17 F16 F15 F14 F13
	bool setFunctions21to28(uint16_t address, uint8_t functions);	//F28 F27 F26 F25 F24 F23 F22 F21
	bool setFunctions29to36(uint16_t address, uint8_t functions);	//F29-F36
	bool setFunctions37to44(uint16_t address, uint8_t functions);	//F37-F44
	bool setFunctions45to52(uint16_t address, uint8_t functions);	//F45-52
	bool setFunctions53to60(uint16_t address, uint8_t functions);	//F53-60
	bool setFunctions61to68(uint16_t address, uint8_t functions);	//F61-68

	byte getFunktion0to4(uint16_t address);	//gibt Funktionszustand - F0 F4 F3 F2 F1 zurück
	byte getFunktion5to8(uint16_t address);	//gibt Funktionszustand - F8 F7 F6 F5 zurück
	byte getFunktion9to12(uint16_t address);	//gibt Funktionszustand - F12 F11 F10 F9 zurück
	byte getFunktion13to20(uint16_t address);	//gibt Funktionszustand F20 - F13 zurück
	byte getFunktion21to28(uint16_t address);	//gibt Funktionszustand F28 - F21 zurück
	byte getFunktion29to31(uint16_t address);	//gibt Funktionszustand F31 - F29 zurück
	
    bool setBasicAccessoryPos(uint16_t address, bool state);
	bool setBasicAccessoryPos(uint16_t address, bool state, bool activ);
	bool getBasicAccessoryInfo(uint16_t address);
	bool setExtAccessoryPos(uint16_t address, uint8_t state);
	
	bool opsProgDirectCV(uint16_t CV, uint8_t CV_data);			//Direct Mode - Write byte
	bool opsVerifyDirectCV(uint16_t CV, uint8_t CV_data);		//Direct Mode - Verify Byte
	bool opsReadDirectCV(uint16_t CV);	//Direct Mode - Read Bit
    bool opsProgramCV(uint16_t address, uint16_t CV, uint8_t CV_data);			//POM write byte
	bool opsPOMwriteBit(uint16_t address, uint16_t CV, uint8_t Bit_data);		//POM write bit
	bool opsPOMreadCV(uint16_t address, uint16_t CV);							//POM read
	
    //to be called periodically within loop()
    void update(void); //checks queues, puts whatever's pending on the rails via global current_packet. easy-peasy
	
	bool getRailComStatus(void);	//return the actual RailComStatus on the rails
	
	#if defined(GLOBALRAILCOMREADER)
	// public only for easy access by interrupt handlers
	//static inline void handle_RX_interrupt();		//Serial RX Interrupt bearbeiten
	#endif
	
  private:
	#if defined(GLOBALRAILCOMREADER)
	static DCCPacketScheduler *active_object;	//aktuelle aktive Object for interrupt handler	
	void RailComReceive(void);	//Read the incomming RailCom data
	uint8_t RailComID;		//1. Channel RailCom ID and 2 Bit data
	uint8_t RailComData;	//next 6 Bit Data
	#endif
	
	uint8_t TrntFormat;		// The Addressing of BasicAccessory Messages
	uint8_t DCCdefaultSteps; 	//default Speed Steps
	volatile byte railpower = 0xFF;				 // actual state of the power that goes to the rails
	
	uint16_t LASTVAmpSence = 0;			//Save the maximal level of mA on the Rail for ACK detection
	uint16_t COUNTVAmpSence = 0;		//Count the number of ACK detection 

	byte BasicAccessory[AccessoryMax / 8];	//Speicher für Weichenzustände
	NetLok LokDataUpdate[SlotMax];	//Speicher zu widerholdene Lok Daten
	byte LokStsgetSlot(uint16_t adr);		//gibt Slot für Adresse zurück / erzeugt neuen Slot (0..126)
	void LokStsSetNew(byte Slot, uint16_t adr);	//Neue Lok eintragen mit Adresse

	byte slotFullNext;	//if no free slot, override existing slots
	
	uint8_t ProgState;	//Direct CV Zustand
	uint8_t ProgMode;	//Direct CV Modus
	byte ProgReadMode;	//lese-Modus für Direct CV
	byte RSTsRepeat;	//Repeat for Reset start Packet
	byte RSTcRepeat;	//Repeat for Reset contingue Packet
	
	void opsWriteCV(uint16_t CV, uint8_t CV_data);		//Direct CV write
	void opsVerifyCV(uint16_t CV, uint8_t CV_data);		//Direct CV prüfen
	void opsReadCV(uint16_t CV, uint8_t bitToRead, bool bitState = 1);		//Direct CV lesen
	bool opsDecoderReset(uint8_t repeat = RESET_CONT_REPEAT);		//Decoder Reset Packet For all Decoders

    uint8_t packet_counter;	//to not repeat only one queue
    
    DCCEmergencyQueue e_stop_queue;		//for accessory and estop only - repeat between periodic!
    DCCRepeatQueue repeat_queue;			//send direct then add to periodic repeat!
//NEW
	DCCTemporalQueue periodic_refresh_queue;	//special - never stop repeating the paket!

	DCCEmergencyQueue ops_programmming_queue;		//NEW ops programming - repeat directly!
};

//DCCPacketScheduler packet_scheduler;

#if defined (__cplusplus)
extern "C" {
#endif

	extern void notifyLokAll(uint16_t Adr, uint8_t Steps, uint8_t Speed, uint8_t F0, uint8_t F1, uint8_t F2, uint8_t F3) __attribute__((weak));
	extern void notifyTrnt(uint16_t Adr, bool State, bool active) __attribute__((weak));
	extern void notifyExtTrnt(uint16_t Adr, uint8_t) __attribute__((weak));
	
	extern void notifyCVVerify(uint16_t CV, uint8_t value) __attribute__((weak));
	
	extern void notifyCVPOMRead(uint16_t CVAdr, uint8_t value) __attribute__((weak));
	
	extern void notifyRailpower(uint8_t state) __attribute__((weak));
	
	extern uint16_t notifyCurrentSence(void) __attribute__((weak));	 //request the CurrentSence value
	
	extern void notifyCVNack(uint16_t CV) __attribute__((weak));	//no ACK while programming

#if defined (__cplusplus)
}
#endif

#endif //__DCC_COMMANDSTATION_H__
