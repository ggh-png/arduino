#include "ID_Write.h"

void setup() {
  // put your setup code here, to run once:
  Serial3.begin(115200); //波特率115200
  delay(1000);
}


void loop() {
  // put your main code here, to run repeatedly:
  delay(500);
  Serial3.println("servo ID 지정");
  LobotSerialServoSetID(Serial3, 254, 4); 
  delay(500);
  Serial3.println("지정 완료");
}
