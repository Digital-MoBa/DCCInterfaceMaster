/****************************************************************************
* Copyright (C) 2016-2021 Philipp Gahtow
*
* DCC Waveform Timer Configuration
*
* DCC Master Interface can generate DCC Signal 
* either with Timer1 (16-bit) or with Timer2 (8-bit)
* for ARM on Arduino DUE with TC3 = Timer4 (TC1 Channel 0)
* for ESP8266 with Timer1
* for ESP32 with Timer1
****************************************************************************/

#define PREAMBLE_LENGTH 17	//1's for the preamble
#define ADD_LONG_PREAMBLE_LENGTH 4	//additional length for Service Mode Packet
#define RAILCOM_CUTOUT_LENGTH 4		//length the preamble will be cut out when railcom data will transmit

//--------------------------------------------------------------------------------------
/*
NEM 670 – Ausgabe 2013:
Dauer des Teil-Einsbits: t = 58 µs & zulässige Toleranzen +/- 3 µs am Gleis
Dauer des Teil-Nullbits: t ≥ 100 µs, normal: 116µs
*/
/******************************************/
#if defined(__AVR__)  
//CONTROL What Timer we should use:
//#define DCC_USE_TIMER1	//USE 16-bit TIMER1
#undef DCC_USE_TIMER1		//USE 8-bit TIMER2
#endif

/******************************************/
//DCC truth table:

//OFF:
#define DCC_OUTPUT1_OFF() 			{*d1reg &= ~d1bit;}		//LOW
#define DCC_OUTPUT1_OFF_legacy	 	LOW
#define DCC_OUTPUT2_OFF() 			{*d2reg &= ~d2bit;}		//LOW 
#define DCC_OUTPUT2_OFF_legacy	 	LOW

//RailCom:
#define DCC_OUTPUT1_RC() 			{*d1reg &= ~d1bit;}		 //LOW 
#define DCC_OUTPUT1_RC_legacy	 	LOW
#define DCC_OUTPUT2_RC() 			{*d2reg &= ~d2bit;}		//LOW
#define DCC_OUTPUT2_RC_legacy	 	LOW

//DCC LOW:
#define DCC_OUTPUT1_LOW() 			{*d1reg &= ~d1bit;}	  //LOW
#define DCC_OUTPUT1_LOW_legacy	 	LOW
#define DCC_OUTPUT2_LOW() 			{*d2reg |= d2bit;}	 //HIGH
#define DCC_OUTPUT2_LOW_legacy	 	HIGH

//DCC HIGH:
#define DCC_OUTPUT1_HIGH() 			{*d1reg |= d1bit;}	 //HIGH
#define DCC_OUTPUT1_HIGH_legacy	 	HIGH
#define DCC_OUTPUT2_HIGH() 			{*d2reg &= ~d2bit;}	 //LOW
#define DCC_OUTPUT2_HIGH_legacy	 	LOW

/******************************************/
//Arduino DUE DCC Signal generation on TC3 = Timer4 (TC1 Channel 0)
#if defined(__SAM3X8E__)
#define one_count	609		// Calls every 58µs
#define zero_high_count	1050	// Calls every 100µs
//#define zero_low_count	1050	// Calls every 100µs
#define DCC_ARM_TC_TIMER	TC2			//Timer
#define DCC_ARM_TC_CHANNEL	0			//Channel
#define DCC_ARM_MATCH_INT	TC6_IRQn	//Interrupt that will be used
#define DCC_ARM_TC_SIGNAL	TC6_Handler()	//Interrupt Handler
#define DCC_TMR_OUTP_ONE_COUNT() {TC_SetRC(DCC_ARM_TC_TIMER, DCC_ARM_TC_CHANNEL, one_count); }	
#define DCC_TMR_OUTP_ZERO_HIGH() {TC_SetRC(DCC_ARM_TC_TIMER, DCC_ARM_TC_CHANNEL, zero_high_count);}    
//#define DCC_TMR_OUTP_ZERO_LOW()  {TC_SetRC(DCC_ARM_TC_TIMER, DCC_ARM_TC_CHANNEL, zero_low_count);}    

/******************************************/
//ESP8266 DCC Signal generation with Timer1
#elif defined(ESP8266) 
//TIM_DIV1 = 0   -> 80MHz (80 ticks/us - 104857.588us max)
//TIM_DIV16 = 1  -> 5MHz (5 ticks/us - 1677721.4us max)
//TIM_DIV256 = 3 -> 312.5Khz (1 tick = 3.2us - 26843542.4us max)
#define one_count 290		// 290 - Calls every 58µs
#define zero_high_count	501	// 500 - Calls every 100µs
#define zero_low_count	501	// Calls every 100µs
#define DCC_ESP_TIMER_DIV 		TIM_DIV16
#define DCC_ESP_TIMER_SET		TIM_EDGE
#define DCC_ESP_TIMER_LOOP  	TIM_SINGLE
#define DCC_TMR_OUTP_ONE_COUNT() {timer1_write(one_count); last_timer = one_count;}	
#define DCC_TMR_OUTP_ZERO_HIGH() {timer1_write(zero_high_count); last_timer = zero_high_count;}    
#define DCC_TMR_OUTP_ZERO_LOW()  {timer1_write(zero_low_count); last_timer = zero_low_count;}    

/******************************************/
//ESP32 DCC Signal generation with Timer1
#elif defined(ESP32)
#define one_count 58		// 290 - Calls every 58µs
#define zero_high_count	100	// 500 - Calls every 100µs
#define zero_low_count	100	// Calls every 100µs
#define DCC_ESP_TIMER_PRESCALE	80		//prescale the value of the time divider
#define DCC_ESP_TIMER_FLAG		true	//flag true to count on the rising edge, false to count on the falling edge
#define DCC_TMR_OUTP_ONE_COUNT() {timerAlarmWrite(timer, one_count, true); last_timer = one_count;}	
#define DCC_TMR_OUTP_ZERO_HIGH() {timerAlarmWrite(timer, zero_high_count, true); last_timer = zero_high_count;}    
#define DCC_TMR_OUTP_ZERO_LOW()  {timerAlarmWrite(timer, zero_low_count, true); last_timer = zero_low_count;} 

/******************************************/
//USE the Timer1 for the DCC Signal generation on AVR
#elif defined(DCC_USE_TIMER1)
#undef DCC_USE_TIMER2

#define one_count 115 //58us = 115
#define zero_high_count 199 //100us = 199	!old: 116us = 228
#define zero_low_count 199  //100us

#define DCC_TMR_SIGNAL         TIMER1_COMPA_vect
#define DCC_INIT_COMPARATOR    TCCR1A
#define DCC_TMR_CONTROL_REG    TCCR1B
//turn on CTC mode in WGM12 and set CS11 for 8 prescaler in TCCR1B
#define DCC_TMR_CONTROL_SET()  {DCC_TMR_CONTROL_REG = 1 << WGM12 | 1 << CS11;}
#define DCC_TMR_COUNT_REG	   TCNT1
#define DCC_TMR_MATCH_INT()    {TIMSK1 |= (1 << OCIE1A);} //Compare Match Interrupt Enable
#define DCC_TMR_OUTP_CAPT_REG  OCR1A

#define DCC_TMR_OUTP_ONE_COUNT() {OCR1A = OCR1B = one_count;}
#define DCC_TMR_OUTP_ZERO_HIGH() {OCR1A = OCR1B = zero_high_count;}
#define DCC_TMR_OUTP_ZERO_LOW()  {OCR1A = OCR1B = zero_low_count;}

/******************************************/
//USE the Timer2 for the DCC Signal generation on AVR
#else
#define DCC_USE_TIMER2

#define one_count 141  //58usec pulse length = 141 = 0x8D
#define zero_high_count 56  //100us = 56		!old: 116us = 0x1B (27) pulse length
#define zero_low_count 56  //100us

#define DCC_TMR_SIGNAL         TIMER2_OVF_vect
#define DCC_INIT_COMPARATOR    TCCR2A
#define DCC_TMR_CONTROL_REG    TCCR2B
//Timer2 Settings: Timer Prescaler /8, mode 0
//Timmer clock = 16MHz/8 = 2MHz oder 0,5usec
#define DCC_TMR_CONTROL_SET()  {DCC_TMR_CONTROL_REG = 0 << CS22 | 1 << CS21 | 0 << CS20;}
#define DCC_TMR_COUNT_REG	   TCNT2
#define DCC_TMR_MATCH_INT()    {TIMSK2 = 1 << TOIE2;} //Overflow Interrupt Enable

//note that there is a latency so take the last time of Timer2 also:
#define DCC_TMR_OUTP_ONE_COUNT() {DCC_TMR_COUNT_REG = DCC_TMR_COUNT_REG + one_count; last_timer = one_count;}
#define DCC_TMR_OUTP_ZERO_HIGH() {DCC_TMR_COUNT_REG = DCC_TMR_COUNT_REG + zero_high_count; last_timer = zero_high_count;}
#define DCC_TMR_OUTP_ZERO_LOW() {DCC_TMR_COUNT_REG = DCC_TMR_COUNT_REG + zero_low_count; last_timer = zero_low_count;}

/******************************************/
#endif