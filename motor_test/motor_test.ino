
const int dirPinA = 4;
const int pwmPinA = 5;

void setup() {
  pinMode(dirPinA, OUTPUT); // motor 방향 설정 핀
  pinMode(pwmPinA, OUTPUT); //motor 속도 설정 핀
}
void loop() {

  digitalWrite(dirPinA, HIGH); // 전진
  analogWrite(pwmPinA,127);  // 속도 0 ~ 255
  delay(500);

  digitalWrite(dirPinA, LOW); // 후진
  analogWrite(pwmPinA,127); // 속도 0~255
  delay(500);    
}
