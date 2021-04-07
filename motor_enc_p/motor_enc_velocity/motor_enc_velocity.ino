// encoder pin
#define encoderPinB  2
#define encoderPinA  3

// motor control pin
#define motorDirPin  4 //
#define motorPWMPin  5 //

#define rotation 4320

// P control
#define Kp 3// P-gain 비례 3이 적당 


int encoderPos = 0; //초기 


unsigned long t_k = 0, t_k_1 = 0, dt = 0;
int encoderPos_k = 0; // 현재 
int encoderPos_k_1 = 0; // 과거 
int d_encoderPos = 0;// 진행 

long m1_speed = 0;// 현 속도 
long m1_err_spd = 0;// 현 속도와 목표 속도의 차  
long control_spd = 0;// 폐루프 조정  



void doEncoderA();
void doEncoderB();
void vel_p(int m1_ref_spd);
void doMotor(bool dir, int vel);



void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);
 
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderB, CHANGE);
 
  pinMode(motorDirPin, OUTPUT); 
  pinMode(motorPWMPin, OUTPUT);

  digitalWrite(motorDirPin, LOW); //LOW --> CCW
  analogWrite(motorPWMPin, 0);

  Serial.begin(57600);
}



void loop(){
  vel_p(30);
}



void doEncoderA(){  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?1:-1;}
void doEncoderB(){  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?-1:1;}



void doMotor(bool dir, long vel){
  digitalWrite(motorDirPin, dir);
  vel = abs(vel);
  long ipwm_u = vel > 255 ? 255 : vel; 
  analogWrite(motorPWMPin, dir?ipwm_u:ipwm_u);
}
 
void vel_p(int m1_ref_spd){

  
  if(Serial.available()){// 시리얼 연결이 되면
    m1_ref_spd = Serial.parseInt();
  }
  t_k = millis();//시간측정 
  dt = t_k - t_k_1;//시간변화율
  if(dt >= 50){
    encoderPos_k = encoderPos; 
    d_encoderPos = encoderPos_k - encoderPos_k_1;// 엔코더 값
    m1_speed = d_encoderPos * 60000/rotation;
    m1_speed /= (signed long)dt;// 거 = 속 시 RPM 값
    t_k_1 = t_k;//시간 초기화
    encoderPos_k_1 = encoderPos_k;//엔코더 초기화
    //Error
    
    m1_err_spd =  m1_ref_spd - m1_speed;
    //P-Controller
    control_spd = Kp * m1_err_spd;//목표 속도가 10일때 100
    // 속도 오차가 생기면 
    
    doMotor(control_spd >= 0 ? HIGH : LOW, control_spd);

    Serial.print("speed ");
    Serial.println(m1_speed);
    Serial.print("오차율 ");
    Serial.println(m1_ref_spd);
  }  
}
