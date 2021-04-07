// encoder pin
const int encoderPinA = 2;
const int encoderPinB = 3;
int encoderPos = 0;

void doEncoderA();
void doEncoderB();


void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
 
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);
 
  Serial.begin(115200);
}

void loop() {
}


void doEncoderA(){ // 빨녹일 때
//  if(digitalRead(encoderPinA)==digitalRead(encoderPinB)) // 같으면
//    encoderPos++; // 정회전
//  else // 다르면
//    encoderPos--; // 역회전
  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?1:-1;
  Serial.print("A   ");
  Serial.println(encoderPos);
}



void doEncoderB(){ // 보파일 때
//  if(digitalRead(encoderPinA)==digitalRead(encoderPinB)) // 같으면
//    encoderPos--; // 역회전
//  else // 다르면
//    encoderPos++; // 정회전
  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?-1:1;
  Serial.print("B   ");
  Serial.println(encoderPos);
}
