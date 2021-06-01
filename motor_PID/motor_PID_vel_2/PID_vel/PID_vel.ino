#include "PID.h"

void setup() {
  // put your setup code here, to run once:
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(motorDirPin_A, OUTPUT); 
  pinMode(motorDirPin_B, OUTPUT); 
  pinMode(motorPWMPin, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  vel_PID(-60);
}
