/********************
* Creates a minimum DCC command station 
********************/

//#include <DCCPacket.h>
//#include <DCCPacketQueue.h>
#include <DCCPacketScheduler.h>

#define DCCPin 6    //Pin for DCC sginal out
#define ShortPin 41  //Pin to detect Short Circuit
#define NDCCPin 39
const int DetectShortCircuit = 15;    //ms to detect short circuit

DCCPacketScheduler dps;
char speed_byte, old_speed = 0;
byte count = 0;
byte prev_state = 1;
byte F0 = 1;

unsigned long previousMillis = 0;        // will store last updated - rail ok
unsigned long lastMillis = 0;

void setup() {
  Serial.begin(115200);
  dps.setup(DCCPin,NDCCPin,true,ROCO);  //with Railcom

  pinMode(ShortPin, INPUT);	//set short pin
  digitalWrite(ShortPin, HIGH);  //Pull-UP

  Serial.print("Power: ");
  Serial.println(dps.getpower());
  delay(3000);
  dps.setpower(0x00);
  Serial.print("Power: ");
  Serial.println(dps.getpower());

  dps.setSpeed128(3,0); 
}

void loop() {
  
  unsigned long cMillis = millis();
  if (cMillis - lastMillis >= 1000) {
    lastMillis = cMillis;
    if (count % 5 == 0) {
      //toggle!
      F0 ^= 1;
      Serial.print("SET A29, F0: ");
      Serial.println(F0,BIN);
      dps.setLocoFunc(29,F0,0);

      dps.setBasicAccessoryPos(1,F0, true);	//Adr, Status(straight, turnout), activ
      dps.setBasicAccessoryPos(4,F0, true);
    }
    //handle reading throttle
    ++count;
//    Serial.print("SET A22, Speed: ");
//    Serial.println(count);
//    dps.setSpeed128(22,count); 
    if (count > 127)
      count = 0;
  }
  //ShortDetection();  //handel short on rail => power off
  dps.update();
}

void ShortDetection() { 
  unsigned long currentMillis = millis();
  if (digitalRead(ShortPin) == HIGH) {  //Short Circuit!
//    if(currentMillis - previousMillis >= DetectShortCircuit) {
      if (dps.getpower() == true) {
        Serial.println("Short Circuit");
        dps.setpower(false);
      }
  }
  else previousMillis = currentMillis;
}
