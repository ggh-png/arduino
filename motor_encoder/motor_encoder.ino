// encoder pin
const int encoderPinA = 2;
const int encoderPinB = 3;
#define M4_I1       4

int encoderPos = 0;

void doEncoderA();
void doEncoderB();


void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), doEncoderA, CHANGE);
 
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), doEncoderB, CHANGE);
 
  Serial.begin(115200);
}

void loop() {
  //Serial.println("asd");
}


void doEncoderA(){ // 빨녹일 때
//  if(digitalRead(encoderPinA)==digitalRead(encoderPinB)) // 같으면
//    encoderPos++; // 정회전
//  else // 다르면
//    encoderPos--; // 역회전
  encoderPos += (digitalRead(2)==digitalRead(3))?1:-1;
  Serial.print("A   ");
  Serial.println(encoderPos);
}



void doEncoderB(){ // 보파일 때
  //if(digitalRead(encoderPinA)==digitalRead(encoderPinB)) // 같으면
  //  encoderPos--; // 역회전
  //else // 다르면
  //  encoderPos++; // 정회전
  encoderPos += (digitalRead(2)==digitalRead(3))?-1:1;
  Serial.print("B   ");
  Serial.println(encoderPos);
}
