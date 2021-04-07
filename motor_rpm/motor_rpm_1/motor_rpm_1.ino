// encoder pin
#define encoderPinB  2
#define encoderPinA  3


// motor control pin
#define motorDirPin  4 //
#define motorPWMPin  5 //

#define rotation 4320


const float ratio = 360./rotation;

// P control
float Kp = 10;
float targetDeg = 360*3.5 + 22;

void doEncoderA();
void doEncoderB();

unsigned long t_k = 0, t_k_1 = 0, dt = 0; //시간저장변수
int encoderPos = 0;
int encoderPos_k = 0, encoderPos_k_1 = 0, d_encoderPos = 0; //encoder 현재값, 과거값, 증감값 저장변수
signed long m1speed = 0; //motor rpm


void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);
 
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderB, CHANGE);
 
  pinMode(motorDirPin, OUTPUT);
  pinMode(motorPWMPin, OUTPUT);

  Serial.begin(115200);
}





void loop(){

  RPM(50);
  
}




void doEncoderA(){  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?1:-1;}
void doEncoderB(){  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?-1:1;}



void RPM (int vel){
  
  digitalWrite(motorDirPin, LOW); //LOW --> CCW
  analogWrite(motorPWMPin, vel);
  t_k = millis(); //현재시간 받기
  dt = t_k - t_k_1; //현재시간-과거시간 = 측정시
  if(dt >= 50){ //dt 가 50msec 이상일때
    encoderPos_k = encoderPos; // 현재 enc값 초기화 
    d_encoderPos = encoderPos_k - encoderPos_k_1; // 과거 enc - 현재 enc = 경과한 enc
    m1speed = d_encoderPos * 60000/4320; //ms 단위  따라서 1 ms 당 회전량 / 1 rotation enc 값 
    m1speed /= (signed long)dt;

    t_k_1 = t_k;
    encoderPos_k_1 = encoderPos_k; //누적 enc값 초기화
    Serial.print("RPM : "); 
    Serial.println(m1speed);
   }  
}
