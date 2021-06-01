#include "servo.h"

byte newMode; // f:전진, b:후진, l:좌회전, r:우회전, s:정지, a:가속, d:감속
byte preMode;

#define ID1   0
#define ID2   1
#define ID3   3
#define ID4   4


double Servo_To_Degrees(float position_radians);

void setup() {
  // put your setup code here, to run once:
  Serial3.begin(115200);
  Serial.begin(115200);
  LobotSerialServoMove(Serial3, ID1,Servo_To_Degrees(120), 1000);
  LobotSerialServoMove(Serial3, ID2,Servo_To_Degrees(120), 1000);
  LobotSerialServoMove(Serial3, ID3,Servo_To_Degrees(120), 1000);    
  LobotSerialServoMove(Serial3, ID4,Servo_To_Degrees(48), 1000);  
}


void loop(){
  keyboard_function();
}
