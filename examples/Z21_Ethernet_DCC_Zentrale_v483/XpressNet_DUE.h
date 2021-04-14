#ifdef __SAM3X8E__

#include "soft_uart.h"
using namespace arduino_due;

//Hardware Serial1 interface:
#define RX_PIN 19 // software serial port's reception pin
#define TX_PIN 18 // software serial port's transmision pin
#define SOFT_UART_BIT_RATE 62500 // 57600 38400 1200 19200 9600 115200 115200
#define RX_BUF_LENGTH 256 // software serial port's reception buffer length
#define TX_BUF_LENGTH 256 // software serial port's transmision buffer length
#define RECEPTION_TIMEOUT 100 // milliseconds

// declaration of software serial port object serial_tc4
// which uses timer/counter channel TC4
serial_tc4_declaration(RX_BUF_LENGTH,TX_BUF_LENGTH);
auto& serial_obj=serial_tc4; // serial_tc4_t& serial_obj=serial_tc4;

/*
  // declaration of software serial port object serial_tc4
  // which uses timer/counter channel TC4
  //  serial_tc4_declaration(RX_BUF_LENGTH,TX_BUF_LENGTH);
void TC4_Handler(void) 
{ 
  uint32_t status=TC_GetStatus( 
    arduino_due::soft_uart::tc_timer_table[ 
      static_cast<uint32_t>( 
        arduino_due::soft_uart::timer_ids::TIMER_TC4 
      ) 
    ].tc_p, 
    arduino_due::soft_uart::tc_timer_table[ 
      static_cast<uint32_t>( 
        arduino_due::soft_uart::timer_ids::TIMER_TC4 
      ) 
    ].channel 
  ); 
  
  arduino_due::soft_uart::uart< 
    arduino_due::soft_uart::timer_ids::TIMER_TC4, 
    RX_BUF_LENGTH, 
    TX_BUF_LENGTH 
  >::tc_interrupt(status); 
} 

typedef arduino_due::soft_uart::serial< 
  arduino_due::soft_uart::timer_ids::TIMER_TC4, 
  RX_BUF_LENGTH, 
  TX_BUF_LENGTH 
> serial_tc4_t; 

serial_tc4_t serial_tc4;
 */ 

void XpressNet_DUE_setup() {
  // serial_obj initialization
  // we will communicate serial_obj with itself, so RX_PIN and TX_PIN
  // should be connected. This example illustrate how to use the serial
  // objects provided by soft_uart with a length of 9 bits, because
  // the serial objects Serial, Serial1, Serial2 and Serial3 provided
  // by the standard Arduino library do not provide 9-bit lenght serial
  // modes
  serial_obj.begin(
    RX_PIN,
    TX_PIN,
    SOFT_UART_BIT_RATE,
    soft_uart::data_bit_codes::NINE_BITS,
    soft_uart::parity_codes::NO_PARITY,
    soft_uart::stop_bit_codes::ONE_STOP_BIT
  );
  Serial.println("DUE_Soft_Serial_Init");
}

void XpressNet_DUE_update() {
  if(serial_obj.available()) 
  {
    unsigned int data = serial_obj.read();
    if (data < 0x1FF && data != 0x1FE)
      XpressNet.DUE_serial_read(data); 
  }
}

void XpressNet_DUE_send(uint32_t data) {
  serial_obj.write(data);
  //Serial.print("send: ");
  //Serial.println(data, HEX);
}

#endif
