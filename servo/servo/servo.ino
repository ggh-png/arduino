#include "servo.h"
void setup() {
  // put your setup code here, to run once:
  Serial3.begin(115200);
  Serial.begin(115200);  

}

#define ID1   1
#define ID2   2
#define ID3   3
#define ID4   4
int16_t position = 100;
uint16_t time = 500;
void loop() {
  // put your main code here, to run repeatedly:

    LobotSerialServoMove(Serial3, 4,1000, 1000);
    Serial.println(LobotSerialServoReadPosition(Serial3, 4));
  }


  
