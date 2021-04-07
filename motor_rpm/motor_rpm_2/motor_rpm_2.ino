#include <MsTimer2.h>

// encoder pin
#define encoderPinB  2
#define encoderPinA  3


// motor control pin
#define motorDirPin  4 //
#define motorPWMPin  5 //

#define rotation 4320


const float ratio = 360./rotation;

void doEncoderA();
void doEncoderB();
void RMP(int vel);

int encoderPos = 0;
int encoderPos_k = 0, encoderPos_k_1 = 0, d_encoderPos = 0; //encoder 현재값, 과거값, 증감값 저장변수
signed long m1_speed = 0; //motor rpm
float m1_turn = 0; //motor 회전수
bool t2_flag = 0;


void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);
 
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderB, CHANGE);
 
  pinMode(motorDirPin, OUTPUT);
  pinMode(motorPWMPin, OUTPUT);

  Serial.begin(115200);

  MsTimer2::set(50, T2ISR); // flash 함수를 500ms 마다 호출
  MsTimer2::start();
}


void loop(){
  RPM(100);

}



void RPM(int vel){
    if(t2_flag){
    t2_flag = 0;
    encoderPos_k = encoderPos;// 현 enc = 실행중인 enc 
    d_encoderPos = encoderPos_k - encoderPos_k_1;
    m1_speed = d_encoderPos * 60000/rotation/50; //ms 단위  따라서 1 ms 당 회전량 / 1 rotation enc 값/ 50ms 
    m1_turn = (float)encoderPos_k/rotation; // [pulse]/(11*4*90)
    encoderPos_k_1 = encoderPos_k;
    Serial.print("RMP : ");
    Serial.println(m1_speed);
    Serial.print("turn : ");
    Serial.println(m1_turn);
  }
  digitalWrite(motorDirPin, HIGH); //LOW --> CCW
  analogWrite(motorPWMPin, vel);
}

void T2ISR(){
  t2_flag = 1;
}


void doEncoderA(){  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?1:-1;}
void doEncoderB(){  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?-1:1;}
