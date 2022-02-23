/*
* DCC Waveform Generator
*
* modified by Philipp Gahtow
* Copyright digitalmoba@arcor.de, http://pgahtow.de
*
*/

#include "Arduino.h"

#if defined(__AVR__)
#include <avr/io.h>
#include <avr/interrupt.h>
#endif

#include "DCCHardware.h"
#include "DDCHardware_config.h"

//#define DCCDEBUG	//Serial output of DCC DATA

/// An enumerated type for keeping track of the state machine used in the timer1 ISR
/** Given the structure of a DCC packet, the ISR can be in one of 5 states.
      *dos_idle: there is nothing to put on the rails. In this case, the only legal thing
                 to do is to put a '1' on the rails.  The ISR should almost never be in this state.
      *dos_send_premable: A packet has been made available, and so we should broadcast the preamble with at least: 12 '1's in a row
	  *dos_send_longpreamble: Additional '1's for Service Mode packets
      *dos_send_bstart: Each data uint8_t is preceded by a '0'
      *dos_send_uint8_t: Sending the current data uint8_t
      *dos_end_bit: After the final uint8_t is sent, send a '0'.
*/                 
typedef enum  {
  dos_idle,
  dos_send_preamble,
  dos_send_longpreamble,
  dos_send_bstart,
  dos_send_next_uint8_t,
  dos_send_uint8_t,
  dos_end_bit
} DCC_output_state_t;

volatile DCC_output_state_t DCC_state = dos_idle; //just to start out

/// The Pin where the DCC Waveform comes out.
bool POWER_STATUS = false;	//set the railsignal on/off
uint8_t DCCPin = 6;
uint8_t DCCPin2 = 0xFF;	// inverted DCC (for RailCom support)
bool RailCom = false;	//provide a cut out of four bit in preamble
bool RailComHalfOneBit = false;		//last bit before RC start is only a half one!
uint8_t DCCS88Pin = 0xFF;	//provide all the time a DCC Signal (no RailCom), even when Railpower is off!

#if defined(__AVR__)
//data for DCC direct register setting
uint8_t d1bit;		//_BV Bit for DCC
uint8_t d2bit;		//_BV Bit for DCC2
uint8_t d3bit;		//_BV Bit for S88
volatile uint8_t *d1reg, *d2reg, *d3reg;	//PORT Register
#endif

#if defined(ESP32)
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;	
#endif

volatile bool get_next_packet = true;	//notify to update() that we need the next packet

volatile uint8_t sending_packet[6] = {0,0,0,0,0,0};
volatile uint8_t sending_packet_size = 0;

/// The currently queued packet to be put on the rails. Default is a reset packet.
volatile uint8_t current_packet[6] = {0,0,0,0,0,0};
/// is in Service Mode:
volatile uint8_t current_packet_service = false;	//actual packet is a service mode packet
/// How many data uint8_ts in the queued packet?
volatile uint8_t current_packet_size = 0;
/// How many uint8_ts remain to be put on the rails?
volatile uint8_t sending_uint8_t_counter = 0;
/// How many bits remain in the current data uint8_t/preamble before changing states?
volatile uint8_t current_bit_counter = PREAMBLE_LENGTH; //init to 16 1's for the preamble

/// Timer1 TOP values for one and zero
/** S 9.1 A specifies that '1's are represented by a square wave with a half-period of 58us (valid range: 55-61us)
    and '0's with a half-period of >100us (valid range: 95-9900us)
    Because '0's are stretched to provide DC power to non-DCC locos, we need two zero counters,
     one for the top half, and one for the bottom half.

   Here is how to calculate the timer1 counter values (from ATMega168 datasheet, 15.9.2):
 f_{OC1A} = \frac{f_{clk_I/O}}{2*N*(1+OCR1A)})
 where N = prescalar, and OCR1A is the TOP we need to calculate.
 We know the desired half period for each case, 58us and >100us.
 So:
 for ones:
 58us = (8*(1+OCR1A)) / (16MHz)
 58us * 16MHz = 8*(1+OCR1A)
 58us * 2MHz = 1+OCR1A
 OCR1A = 115

 for zeros:
 100us * 2MHz = 1+OCR1A
 OCR1A = 199
 
 This, we also know that the valid range for stretched-zero operation is something like this:
 9900us = (8*(1+OCR1A)) / (16MHz)
 9900us * 2MHz = 1+OCR1A
 OCR1A = 19799
 
*/

#if defined(DCC_USE_TIMER2)
uint8_t last_timer = one_count;	//last time set to timer
#elif defined(ESP8266) || defined(ESP32)
uint16_t last_timer = one_count;	//last time set to timer
#endif

volatile uint8_t oldstate = LOW;	//state of the output
volatile uint8_t RailComActiv = false;

/// This is the Interrupt Service Routine (ISR) for Timer compare match.
#if defined(__AVR__)
ISR(DCC_TMR_SIGNAL)
#elif defined (ESP8266) || defined (ESP32) 
static void IRAM_ATTR onTimerISR()
#else
void DCC_ARM_TC_SIGNAL
#endif
{
	#if defined(ESP32)
	//portENTER_CRITICAL_ISR(&timerMux);
	#endif
	
	#if defined(__SAM3X8E__)
	TC_GetStatus(DCC_ARM_TC_TIMER, DCC_ARM_TC_CHANNEL);
	#endif
	
  //in CTC mode, timer TCINT1 automatically resets to 0 when it matches OCR1A. Depending on the next bit to output,
  //we may have to alter the value in OCR1A, maybe.
  //to switch between "one" waveform and "zero" waveform, we assign a value to OCR1A.
  
  //remember, anything we set for OCR1A takes effect IMMEDIATELY, so we are working within the cycle we are setting.
  //first, check to see if we're in the second half of a uint8_t; only act on the first half of a uint8_t
/*
	//On Arduino UNO, etc, OC1A is digital pin 9, or Port B/Pin 1
  //On Arduino MEGA, etc, OC1A is digital pin 11, or Port B/Pin 5
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_AT90CAN128__) || defined(__AVR_AT90CAN64__) || defined(__AVR_AT90CAN32__)
  if(PINB & (1<<PINB6)) //if the pin is low, we need to use a different zero counter to enable streched-zero DC operation
#else
  if(PINB & (1<<PINB1)) //if the pin is low, we need to use a different zero counter to enable streched-zero DC operation
#endif
*/

  if (RailComHalfOneBit) {
	//deactivate:
	RailComHalfOneBit = false;
	DCC_TMR_OUTP_ONE_HALF();	//Produce another halve "one" Bit
	#if defined(__AVR__) 
		DCC_OUTPUT1_RC(); //LOW
	#else
		digitalWrite(DCCPin, DCC_OUTPUT1_RC_legacy);
	#endif
	return;	//make the next halve one bit => one bit
  }
  else {	
	oldstate = !oldstate;	//change State
  }
  
  
  if (POWER_STATUS) {	//Railpower ON?
	  if (RailComActiv) {	//Railcom CutOut:
		//Sendepause beträgt mindestens 448µs oder 4 logische 1 Bits (=464µs).
		#if defined(__AVR__)  
			DCC_OUTPUT1_RC(); //LOW
  		    DCC_OUTPUT2_RC(); //LOW
		#else
			digitalWrite(DCCPin, DCC_OUTPUT1_RC_legacy);	
			digitalWrite(DCCPin2, DCC_OUTPUT2_RC_legacy);	
		#endif
	  }
	  else {
		 //normal Working = no RailCom Cutout:
		 if (oldstate == LOW) {
			#if defined(__AVR__)
				DCC_OUTPUT1_LOW(); //LOW
				DCC_OUTPUT2_LOW(); //HIGH
			#else
				digitalWrite(DCCPin, DCC_OUTPUT1_LOW_legacy);	
				digitalWrite(DCCPin2, DCC_OUTPUT2_LOW_legacy);	
			#endif
		  }
		  else {
			#if defined(__AVR__)  
				DCC_OUTPUT1_HIGH(); //HIGH
				DCC_OUTPUT2_HIGH(); //LOW
			#else
				digitalWrite(DCCPin, DCC_OUTPUT1_HIGH_legacy);	
				digitalWrite(DCCPin2, DCC_OUTPUT2_HIGH_legacy);	
			#endif
		  }
	  }
  }
  
  
  //True DCC Output for S88/LocoNet without RailCom cutout!
  if (DCCS88Pin != 0xFF) {
	  if (oldstate == LOW) {
		#if defined(__AVR__)
			*d3reg &= ~d3bit; //LOW
		#else
			digitalWrite(DCCS88Pin, LOW);
		#endif
	  }
	  else {
		#if defined(__AVR__)  
			*d3reg |= d3bit; //HIGH
		#else
			digitalWrite(DCCS88Pin, HIGH);	
		#endif
	  }
  }
  
  //only repeat the last output:
  if (oldstate == LOW)
  {
	  //if the pin is low and outputting a zero, we need to be using zero_low_count
	#if defined(DCC_USE_TIMER1)
	  if (DCC_TMR_OUTP_CAPT_REG == zero_high_count) 
      {
		DCC_TMR_OUTP_ZERO_LOW();
      }
	#elif defined(DCC_USE_TIMER2) || defined (ESP8266) || defined(ESP32)	
	  if (last_timer == zero_high_count) 
	  {
		DCC_TMR_OUTP_ZERO_LOW();
	  }
	  else {
		DCC_TMR_OUTP_ONE_COUNT();
	  }
	#endif
	
	
	//ca. 16µs vor dem Ende des 4. Einsbit wieder ein!
	if ((DCC_state == dos_send_preamble) && current_bit_counter == (PREAMBLE_LENGTH - RAILCOM_CUTOUT_LENGTH)) {
		RailComActiv = false;	//stop railcom cutout with the next circle
	}

  }
  //the pin is high. New cycle is begining. Here's where the real work goes.
  //time to switch things up, maybe. send the current bit in the current packet.
  //if this is the last bit to send, queue up another packet (might be the idle packet).
  else { 
  
    //check the state we are in?
	//change structure to if/else because ESP32/ESP-IDF, compiler bug in switch/case #1330 - IRAM crash (Cache disabled but cached memory region accessed)
	
	/// Idle: Check if a new packet is ready. If it is, fall through to dos_send_preamble. Otherwise just stick a '1' out there.  
	  if (DCC_state == dos_idle || DCC_state == dos_send_preamble) {
		  if (DCC_state == dos_idle) {
			DCC_state = dos_send_preamble; //and fall through to dos_send_preamble
		
			//29µs (+/-3µs) nach dem Aussenden des Endebits einer DCC-Nachricht schaltet die Zentrale ab!
			if ((RailCom) && (!current_packet_service)) { //in Service Mode kein RailCom
				RailComActiv = true;	//start railcom cutout within the next circle
				RailComHalfOneBit = true;		//next Bit has only halve length
			}
		  }
      /// Preamble: In the process of producing 16 '1's, counter by current_bit_counter; when complete, move to dos_send_bstart or long preamble
		if (RailComHalfOneBit) {
			DCC_TMR_OUTP_ONE_HALF();
		}
		else {
			DCC_TMR_OUTP_ONE_COUNT();
		}
		  #if defined(DCCDEBUG)
			Serial.print("P");	
		  #endif		
        if(!--current_bit_counter) {
			if (current_packet_service == true) { //long Preamble in Service Mode
				current_bit_counter = ADD_LONG_PREAMBLE_LENGTH;	//additional '1's
				DCC_state = dos_send_longpreamble;
			}
			else DCC_state = dos_send_bstart;
		}
		//break;
	  }
	/// long Preamble: producess additional '1's for Service Mode data		
    else if (DCC_state == dos_send_longpreamble) {
		DCC_TMR_OUTP_ONE_COUNT();
		#if defined(DCCDEBUG)
		Serial.print("L");	
		#endif	
		if (!--current_bit_counter) {
			DCC_state = dos_send_bstart;
		}
		//break;
	  }
    /// About to send a data uint8_t, but have to peceed the data with a '0'. Send that '0', then move to dos_send_uint8_t
	else if (DCC_state == dos_send_bstart) {
		DCC_TMR_OUTP_ZERO_HIGH();
		#if defined(DCCDEBUG)
        Serial.print(" 0 ");
		#endif
		//check if we have received a next packet to send?
		if (get_next_packet) {	//ERROR! - We didn't get the next packet until now!
			//load a default idle packet!
			sending_packet[0] = 0xFF;
			sending_packet[1] = 0;
			sending_packet[2] = 0xFF;
			sending_packet_size = 3;  //feed to the starting ISR.
		}
		//store the current_packet to let time for getting the next one while sending this.
		else {
			sending_packet[0] = current_packet[0];
			sending_packet[1] = current_packet[1];
			sending_packet[2] = current_packet[2];
			sending_packet[3] = current_packet[3];
			sending_packet[4] = current_packet[4];
			sending_packet[5] = current_packet[5];
			sending_packet_size = current_packet_size;
		}
		get_next_packet = true;
		DCC_state = dos_send_uint8_t;
        current_bit_counter = 8;		//reset the counter for bit sending
		sending_uint8_t_counter	= sending_packet_size;	//reset the counter to the packet_size
		//break;
	}
    /// About to send next data uint8_t, but have to peceed the data with a '0'. Send that '0', then move to dos_send_uint8_t
	else if (DCC_state == dos_send_next_uint8_t) {
		DCC_TMR_OUTP_ZERO_HIGH();
        DCC_state = dos_send_uint8_t;	//continue sending...
        current_bit_counter = 8;	//reset the counter for bit sending
		//break;
	}
    /// Sending a data uint8_t; current bit is tracked with current_bit_counter, and current uint8_t with sending_uint8_t_counter
	else if (DCC_state == dos_send_uint8_t) {
        if(((sending_packet[sending_packet_size-sending_uint8_t_counter])>>(current_bit_counter-1)) & 1) //is current bit a '1'?
        {
			DCC_TMR_OUTP_ONE_COUNT();
			#if defined(DCCDEBUG)
            Serial.print("1");
			#endif
        }
        else //or is it a '0'
        {
			DCC_TMR_OUTP_ZERO_HIGH();
			#if defined(DCCDEBUG)
            Serial.print("0");
			#endif
        }
        if(!--current_bit_counter) //out of bits! time to either send a new uint8_t, or end the packet
        {
          if(!--sending_uint8_t_counter) //if not more uint8_ts, move to dos_end_bit
          {
            DCC_state = dos_end_bit;
          }
          else //there are more uint8_ts…so, go back to dos_send_bstart
          {
            DCC_state = dos_send_next_uint8_t;
          }
        }
		//break;
	}
    /// Done with the packet. Send out a final '1', then head back to dos_idle to check for a new packet.
	else if (DCC_state == dos_end_bit) {
		DCC_TMR_OUTP_ONE_COUNT();
		DCC_state = dos_idle;
		current_bit_counter = PREAMBLE_LENGTH; //in preparation for a premable...
		#if defined(DCCDEBUG)
        Serial.println(" 1");
		#endif
		//break;
	}
	//} //END SWITCH CASE
	
  }  //END the pin is high.
  
  #if defined(ESP32)
	//portEXIT_CRITICAL_ISR(&timerMux);
  #endif
}

void DCC_stop_output_signal()
{
	POWER_STATUS = false;
	digitalWrite(DCCPin, LOW);	//DCC output pin inaktiv
	if (DCCPin2 != 0xFF) {
		digitalWrite(DCCPin2, LOW);	//DCC output pin inaktiv
	}
}

void DCC_run_output_signal()
{
	POWER_STATUS = true;
}

/// Setup phase: configure and enable timer1 CTC interrupt, set OC1A and OC1B to toggle on CTC
void setup_DCC_waveform_generator() {

/*
 //Set the OC1A and OC1B pins (Timer1 output pins A and B) to output mode
 //On Arduino UNO, etc, OC1A is Port B/Pin 1 and OC1B Port B/Pin 2
 //On Arduino MEGA, etc, OC1A is or Port B/Pin 5 and OC1B Port B/Pin 6
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_AT90CAN128__) || defined(__AVR_AT90CAN64__) || defined(__AVR_AT90CAN32__)
  DDRB |= (1<<DDB5) | (1<<DDB6);
#else
  DDRB |= (1<<DDB1) | (1<<DDB2);
#endif

  // Configure timer1 in CTC mode, for waveform generation, set to toggle OC1A, OC1B, at /8 prescalar, interupt at CTC
  TCCR1A = (0<<COM1A1) | (1<<COM1A0) | (0<<COM1B1) | (1<<COM1B0) | (0<<WGM11) | (0<<WGM10);
  TCCR1B = (0<<ICNC1)  | (0<<ICES1)  | (0<<WGM13)  | (1<<WGM12)  | (0<<CS12)  | (1<<CS11) | (0<<CS10);

  //finally, force a toggle on OC1B so that pin OC1B will always complement pin OC1A
  TCCR1C |= (1<<FOC1B);
  
*/
	pinMode(DCCPin, OUTPUT);		//Set output mode for DCC Pin
		
	/******************************************/
	#if defined(__AVR__)
	d1bit = digitalPinToBitMask(DCCPin);
	d1reg = portOutputRegister(digitalPinToPort(DCCPin));	//PORTB, PORTC, ....
	DCC_OUTPUT1_OFF(); //LOW
	
    pinMode(DCCPin2, OUTPUT);		//Set output mode for DCC Pin2
	d2bit = digitalPinToBitMask(DCCPin2);
	d2reg = portOutputRegister(digitalPinToPort(DCCPin2));
	DCC_OUTPUT2_OFF(); //LOW

	
	if (DCCS88Pin != 0xFF) {
		pinMode(DCCS88Pin, OUTPUT);		//Set output mode for DCC S88 Pin
		d3bit = digitalPinToBitMask(DCCS88Pin);
		d3reg = portOutputRegister(digitalPinToPort(DCCS88Pin));
		*d3reg &= ~d3bit; //LOW
	}
		
	DCC_INIT_COMPARATOR = 0;     // set entire TCCR1A register to 0
	DCC_TMR_CONTROL_REG = 0;     // same for TCCR1B
		
	// start by outputting a '1'
	DCC_TMR_OUTP_ONE_COUNT(); //Whenever we set OCR1A, we must also set OCR1B, or else pin OC1B will get out of sync with OC1A!

	DCC_TMR_COUNT_REG = 0; //get the timer rolling (not really necessary? defaults to 0. Just in case.)

	DCC_TMR_CONTROL_SET();  //Timer Prescaler and mode set
	
	/******************************************/
	/******************************************/
	#else  //other

	digitalWrite(DCCPin, DCC_OUTPUT1_OFF_legacy);

	pinMode(DCCPin2, OUTPUT);		//Set output mode for DCC Pin2
	digitalWrite(DCCPin2, DCC_OUTPUT2_OFF_legacy);
	
	if (DCCS88Pin != 0xFF) {
		pinMode(DCCS88Pin, OUTPUT);		//Set output mode for DCC S88 Pin
		digitalWrite(DCCS88Pin, LOW);
	}
	
	/******************************************/
	#if defined(ESP32)			//ESP32 Modul
	/* Use 1st timer of 4 */
	/* 1 tick take 1/(80MHZ/80) = 1us so we set divider 80 and count up */
	timer = timerBegin(DCC_ESP_TIMER_ID, DCC_ESP_TIMER_PRESCALE, DCC_ESP_TIMER_FLAG);
	
	/* Set alarm to call onTimer function every second 1 tick is 1us => 1 second is 1000000us */
	/* Repeat the alarm (third parameter) */
	timerAttachInterrupt(timer, &onTimerISR, true);
	

	DCC_TMR_OUTP_ONE_COUNT(); //start output "1"

	/******************************************/		
	#elif defined (ESP8266)		//ESP8266
	noInterrupts();
	timer1_isr_init();
	timer1_enable(DCC_ESP_TIMER_DIV, DCC_ESP_TIMER_SET, DCC_ESP_TIMER_LOOP);
	
	DCC_TMR_OUTP_ONE_COUNT(); //start output "1"
	interrupts();
	
	/******************************************/
	#else						//Arduino DUE
	// Tell the Power Management Controller to disable 
	// the write protection of the (Timer/Counter) registers:
	pmc_set_writeprotect(false);
	
	// Enable clock for the timer
	pmc_enable_periph_clk((uint32_t)DCC_ARM_MATCH_INT);
	
	// Set up the Timer in waveform mode which creates a PWM
	// in UP mode with automatic trigger on RC Compare
	// and sets it up with the determined internal clock as clock input.
	TC_Configure(DCC_ARM_TC_TIMER, DCC_ARM_TC_CHANNEL, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2);
	
	// Set up timing...
	DCC_TMR_OUTP_ONE_COUNT();
	
	// Enable the RC Compare Interrupt...
	DCC_ARM_TC_TIMER->TC_CHANNEL[DCC_ARM_TC_CHANNEL].TC_IER=TC_IER_CPCS;
	
	// ... and disable all others.
	DCC_ARM_TC_TIMER->TC_CHANNEL[DCC_ARM_TC_CHANNEL].TC_IDR=~TC_IER_CPCS;
	
	#endif
	
	/******************************************/
	#endif
	
	/******************************************/
	//Enable the Interrupt (all MCUs):
	
	#if defined(__AVR__)
	//enable match interrupt
	DCC_TMR_MATCH_INT();
	
	#elif defined(ESP8266) //ESP8266
	timer1_attachInterrupt(onTimerISR);
	
	#elif defined(ESP32)	//ESP32
	timerAlarmEnable(timer);
	
	#else	//Arduino DUE
	NVIC_EnableIRQ(DCC_ARM_MATCH_INT);	
	#endif
}