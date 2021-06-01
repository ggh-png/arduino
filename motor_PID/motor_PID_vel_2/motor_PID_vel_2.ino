// encoder pin
#define encoderPinB  2
#define encoderPinA  A1

// motor control pin
#define motorDirPin_A  A4 //
#define motorDirPin_B  A5 //
#define motorPWMPin  5 //

#define rotation 1152
// 12 * 48 * 2
// PID control
#define Kp 0.6// P-gain 
#define Ki 0.05// i-gain  
#define Kd 0.01// d-gain 

int encoderPos = 0; //초기 
long ipwm_u = 0;

unsigned long t_k = 0, t_k_1 = 0, dt = 0;
int encoderPos_k = 0; // 현재 
int encoderPos_k_1 = 0; // 과거 
int d_encoderPos = 0;// 진행 

long m1_speed = 0;// 현 속도 
long m1_err_spd = 0;// 현 속도와 목표 속도의 차  
long control_spd = 0;// 폐루프 조정  



void doEncoderA();
void doEncoderB();
void RPM(int m1_ref_spd);
void doMotor(bool dir, int vel);
void vel_PID(int m1_ref_spd);


void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(motorDirPin_A, OUTPUT); 
  pinMode(motorDirPin_B, OUTPUT); 
  pinMode(motorPWMPin, OUTPUT);
  Serial.begin(115200);
}



void loop(){

  vel_PID(-60);
}



void doEncoderA(){  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?1:-1;}

void doMotor(bool dir, long vel){
  
  digitalWrite(motorDirPin_A, dir);
  digitalWrite(motorDirPin_B, !dir);
  vel = abs(vel);
  ipwm_u = vel > 255 ? 255 : vel; 
  analogWrite(motorPWMPin, ipwm_u);
}


void RPM(int m1_ref_spd){
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
  }
}


int err_P=0;
int err_I=0;
int err_D=0;
int err_B=0;
long PID_val = 0;


void vel_PID(int m1_ref_spd){

    RPM(m1_ref_spd);
    m1_err_spd =  m1_ref_spd - m1_speed;
    err_P = m1_err_spd;
    err_I += err_P;
    err_D = err_B - err_P;
    err_B = err_P;

    PID_val=((err_P * Kp) + (err_I * Ki) + (err_D * Kd));

    //P-Controller
    control_spd = PID_val;
    // 속도 오차가 생기면 
    
    doMotor(control_spd >= 0 ? HIGH : LOW, control_spd);

    Serial.print("목표속도 : ");
    Serial.println(m1_ref_spd);
    Serial.print("현재속도 : ");
    Serial.println(m1_speed);
    
}
