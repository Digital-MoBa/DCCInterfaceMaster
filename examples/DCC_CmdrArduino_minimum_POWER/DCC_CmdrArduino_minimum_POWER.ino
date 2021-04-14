/********************
* Creates a minimum DCC command station 
********************/

#include <DCCPacketScheduler.h>

#define DEBUG Serial
#define POWER   //Switch the RailPower ON and OFF!

#if defined(__arm__) && defined(DEBUG)
#define DEBUG Serial //SerialUSB for SAM
#endif

#define DCCPin 6    //(Pin 6) for DCC sginal out
#define ShortPin 41 //(Pin 41) to detect Short Circuit
#define NDCCPin 39  //(Pin 39)
const int DetectShortCircuit = 15;    //ms to detect short circuit

DCCPacketScheduler dps;
char speed_byte, old_speed = 0;
byte count = 0;
byte prev_state = 1;
byte F0 = 1;

unsigned long previousMillis = 0;        // will store last updated - rail ok
unsigned long lastMillis = 0;

void setup() {
  DEBUG.begin(115200);
  dps.setup(DCCPin,NDCCPin,DCC128,ROCO);  //with Railcom
  dps.enable_additional_DCC_output(11);
  dps.setCurrentLoadPin(A0);
  dps.setRailcom(true);

  pinMode(ShortPin, INPUT);	//set short pin
  digitalWrite(ShortPin, HIGH);  //Pull-UP

  DEBUG.print("Power: ");
  DEBUG.println(dps.getPower());
  delay(3000);
  dps.setPower(0x00);
  DEBUG.print("Power: ");
  DEBUG.println(dps.getPower());

  dps.setSpeed128(15,B10000000); 
}

void loop() {
  
  dps.update();

  //delay(100);
  
  unsigned long cMillis = millis();
  if (cMillis - lastMillis >= 500) {
    lastMillis = cMillis;
    if (count % 5 == 0) {
      
      if (count % 10 == 0) {
        //toggle!
        F0 ^= 1;
        DEBUG.print("SET A15, F0: ");
        DEBUG.println(F0,BIN);
        dps.setLocoFunc(3,F0,0);
      }
 
    }
    #if defined(POWER)
    if (count % 25 == 0) {
      if (dps.getPower() == 0) {
        dps.setPower(2);  //off  
      }
      else dps.setPower(0);  //on
      DEBUG.print("Power: ");
      DEBUG.println(dps.getPower());
    }
    #endif
    
    
     ++count;

     
    /*
    DEBUG.print("SET A6221, Speed: ");
    DEBUG.println(count);
    dps.setSpeed128(6221,count); 
    if (count > 127)
      count = 0;
     */ 
  }
  //ShortDetection();  //handel short on rail => power off
  
}

void ShortDetection() { 
  unsigned long currentMillis = millis();
  if (digitalRead(ShortPin) == HIGH) {  //Short Circuit!
//    if(currentMillis - previousMillis >= DetectShortCircuit) {
      if (dps.getPower() == true) {
        DEBUG.println("Short Circuit");
        dps.setPower(false);
      }
  }
  else previousMillis = currentMillis;
}
