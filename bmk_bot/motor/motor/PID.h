
#define rotation 2160

//for Encoder pin define
#define ENC1_CHA    18  //INT.5
#define ENC1_CHB    31

#define ENC2_CHA    19  //INT.4
#define ENC2_CHB    38

#define ENC3_CHA    3   //INT.1
#define ENC3_CHB    49

#define ENC4_CHA    2   //INT.0
#define ENC4_CHB    A1

//for Motor I/O pin define
#define M1_I1       34
#define M1_I2       35
#define M1_PWM      12

#define M2_I1       37
#define M2_I2       36
#define M2_PWM      8

#define M3_I1       42
#define M3_I2       43
#define M3_PWM      9

#define M4_I1       A5
#define M4_I2       A4
#define M4_PWM      5



#define Kp1 0.4// P-gain 
#define Ki1 0.03// i-gain  
#define Kd1 0.2// d-gain 

#define Kp2 0.4// P-gain 
#define Ki2 0.03// i-gain  
#define Kd2 0.2// d-gain 

#define Kp3 0.4// P-gain 
#define Ki3 0.03// i-gain  
#define Kd3 0.2// d-gain 

#define Kp4 0.4// P-gain 
#define Ki4 0.03// i-gain  
#define Kd4 0.2// d-gain 

//Variables
//Encoder value
long    e1cnt = 0;
long    e1cnt_k = 0, e1cnt_k_1 = 0, d_e1cnt = 0;
long    e2cnt = 0;
long    e2cnt_k = 0, e2cnt_k_1 = 0, d_e2cnt = 0;
long    e3cnt = 0;
long    e3cnt_k = 0, e3cnt_k_1 = 0, d_e3cnt = 0;
long    e4cnt = 0;
long    e4cnt_k = 0, e4cnt_k_1 = 0, d_e4cnt = 0;

//motor value
float   m1_speed = 0;
float   m1turn = 0;
float   m2_speed = 0;
float   m2turn = 0;
float   m3_speed = 0;
float   m3turn = 0;
float   m4_speed = 0;
float   m4turn = 0;

//for motor control variable
//motor1
float   m1_ref_spd = 0;
float   m1_err_spd = 0;
float   m1_err_spd_k_1 = 0;
float   m1_derr_spd = 0;
float   m1_err_sum = 0;
float   m1_ctrl_up = 0;
float   m1_ctrl_ui = 0;
float   m1_ctrl_ud = 0;
int     m1_ctrl_u = 0;
int     m1_ipwm_u = 0;
//motor2
float   m2_ref_spd = 0;
float   m2_err_spd = 0;
float   m2_err_spd_k_1 = 0;
float   m2_derr_spd = 0;
float   m2_err_sum = 0;
float   m2_ctrl_up = 0;
float   m2_ctrl_ui = 0;
float   m2_ctrl_ud = 0;
int     m2_ctrl_u = 0;
int     m2_ipwm_u = 0;
//motor3
float   m3_ref_spd = 0;
float   m3_err_spd = 0;
float   m3_err_spd_k_1 = 0;
float   m3_derr_spd = 0;
float   m3_err_sum = 0;
float   m3_ctrl_up = 0;
float   m3_ctrl_ui = 0;
float   m3_ctrl_ud = 0;
int     m3_ctrl_u = 0;
int     m3_ipwm_u = 0;
//motor4
float   m4_ref_spd = 0;
float   m4_err_spd = 0;
float   m4_err_spd_k_1 = 0;
float   m4_derr_spd = 0;
float   m4_err_sum = 0;
float   m4_ctrl_up = 0;
float   m4_ctrl_ui = 0;
float   m4_ctrl_ud = 0;
int     m4_ctrl_u = 0;
int     m4_ipwm_u = 0;

//-----------------------------------//
float   M1err_P = 0;
float   M1err_I = 0;
float   M1err_D = 0;
float   M1err_B = 0;
long    M1PID_val = 0;

float   M2err_P = 0;
float   M2err_I = 0;
float   M2err_D = 0;
float   M2err_B = 0;
long    M2PID_val = 0;

float   M3err_P = 0;
float   M3err_I = 0;
float   M3err_D = 0;
float   M3err_B = 0;
long    M3PID_val = 0;

float   M4err_P = 0;
float   M4err_I = 0;
float   M4err_D = 0;
float   M4err_B = 0;
long    M4PID_val = 0;

unsigned long M1t_k = 0, M1t_k_1 = 0, M1dt = 0;
unsigned long M2t_k = 0, M2t_k_1 = 0, M2dt = 0;
unsigned long M3t_k = 0, M3t_k_1 = 0, M3dt = 0;
unsigned long M4t_k = 0, M4t_k_1 = 0, M4dt = 0;



void Enc1chA_ISR();
void Enc2chA_ISR();
void Enc3chA_ISR();
void Enc4chA_ISR();


void M1vel_PID(int m1_ref_spd);
void M2vel_PID(int m2_ref_spd);
void M3vel_PID(int m3_ref_spd);
void M4vel_PID(int m4_ref_spd);

void M1RPM(int m1_ref_spd);
void M2RPM(int m2_ref_spd);
void M3RPM(int m3_ref_spd);
void M4RPM(int m4_ref_spd);


void M1doMotor(bool dir, long vel);
void M2doMotor(bool dir, long vel);
void M3doMotor(bool dir, long vel);
void M4doMotor(bool dir, long vel);






void motor_setting(){
    // put your setup code here, to run once:
  pinMode(ENC1_CHA, INPUT_PULLUP);
  pinMode(ENC1_CHB, INPUT_PULLUP);
  pinMode(ENC2_CHA, INPUT_PULLUP);
  pinMode(ENC2_CHB, INPUT_PULLUP);
  pinMode(ENC3_CHA, INPUT_PULLUP);
  pinMode(ENC3_CHB, INPUT_PULLUP);
  pinMode(ENC4_CHA, INPUT_PULLUP);
  pinMode(ENC4_CHB, INPUT_PULLUP);

  pinMode(M1_I1, OUTPUT);
  pinMode(M1_I2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_I1, OUTPUT);
  pinMode(M2_I2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M3_I1, OUTPUT);
  pinMode(M3_I2, OUTPUT);
  pinMode(M3_PWM, OUTPUT);
  pinMode(M4_I1, OUTPUT);
  pinMode(M4_I2, OUTPUT);
  pinMode(M4_PWM, OUTPUT);

  //encoder 값을 받기 위한 외부 인터럽트 설정
  attachInterrupt(digitalPinToInterrupt(ENC1_CHA), Enc1chA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_CHA), Enc2chA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_CHA), Enc3chA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4_CHA), Enc4chA_ISR, CHANGE);
  Serial.begin(115200);
}


void M1vel_PID(int m1_ref_spd){
    if(Serial.available()){
      m1_ref_spd = Serial.parseFloat();
    }

    M1RPM(m1_ref_spd);
    m1_err_spd =  m1_ref_spd - m1_speed;
    M1err_P = m1_err_spd;
    M1err_I += M1err_P;
    M1err_D = M1err_B - M1err_P;
    M1err_B = M1err_P;

    M1PID_val=((M1err_P * Kp1) + (M1err_I * Ki1) + (M1err_D * Kd1));
    m1_ctrl_u = M1PID_val;
    // 속도 오차가 생기면  
    M1doMotor(m1_ctrl_u >= 0 ? HIGH : LOW, m1_ctrl_u);
    Serial.print("m1 speed : ");
    //Serial.println(m1_ref_spd);
    Serial.print(m1_speed);
    Serial.print(" | ");
    
}


void M2vel_PID(int m2_ref_spd){


    if(Serial.available()){
      m2_ref_spd = Serial.parseFloat();
    }

    M2RPM(m2_ref_spd);
    m2_err_spd =  m2_ref_spd - m2_speed;
    M2err_P = m2_err_spd;
    M2err_I += M2err_P;
    M2err_D = M2err_B - M2err_P;
    M2err_B = M2err_P;

    M2PID_val=((M2err_P * Kp2) + (M2err_I * Ki2) + (M2err_D * Kd2));
    m2_ctrl_u = M2PID_val;
    // 속도 오차가 생기면  
    M2doMotor(m2_ctrl_u >= 0 ? HIGH : LOW, m2_ctrl_u);
    Serial.print("m2 speed : ");
    Serial.print(m2_speed);
    Serial.print(" | ");
    
}

void M3vel_PID(int m3_ref_spd){

    if(Serial.available()){
      m3_ref_spd = Serial.parseFloat();
    }


    M3RPM(m3_ref_spd);
    m3_err_spd =  m3_ref_spd - m3_speed;
    M3err_P = m3_err_spd;
    M3err_I += M3err_P;
    M3err_D = M3err_B - M3err_P;
    M3err_B = M3err_P;

    M3PID_val=((M3err_P * Kp3) + (M3err_I * Ki3) + (M3err_D * Kd3));
    m3_ctrl_u = M3PID_val;
    // 속도 오차가 생기면  
    M3doMotor(m3_ctrl_u >= 0 ? HIGH : LOW, m3_ctrl_u);
    Serial.print("m3 speed : ");
    Serial.print(m3_speed);
    Serial.print(" | ");
}

void M4vel_PID(int m4_ref_spd){

    if(Serial.available()){
      m4_ref_spd = Serial.parseFloat();
    }

    M4RPM(m4_ref_spd);
    m4_err_spd =  m4_ref_spd - m4_speed;
    M4err_P = m4_err_spd;
    M4err_I += M4err_P;
    M4err_D = M4err_B - M4err_P;
    M4err_B = M4err_P;

    M4PID_val=((M4err_P * Kp4) + (M4err_I * Ki4) + (M4err_D * Kd4));
    m4_ctrl_u = M4PID_val;
    // 속도 오차가 생기면  
    M4doMotor(m4_ctrl_u >= 0 ? HIGH : LOW, m4_ctrl_u);
    Serial.print("m4 speed : ");
    Serial.println(m4_speed);
}

void M1RPM(int m1_ref_spd){
  
  M1t_k = millis();//시간측정 
  M1dt = M1t_k - M1t_k_1;//시간변화율
  if(M1dt >= 50){
    e1cnt_k = e1cnt; 
    d_e1cnt = e1cnt_k - e1cnt_k_1;// 엔코더 값
    m1_speed = d_e1cnt * 60000/rotation;
    m1_speed /= (signed long)M1dt;// 거 = 속 시 RPM 값
    M1t_k_1 = M1t_k;//시간 초기화
    e1cnt_k_1 = e1cnt_k;//엔코더 초기화
  }
}

void M2RPM(int m2_ref_spd){
  
  M2t_k = millis();//시간측정 
  M2dt = M2t_k - M2t_k_1;//시간변화율
  if(M2dt >= 50){
    e2cnt_k = e2cnt; 
    d_e2cnt = e2cnt_k - e2cnt_k_1;// 엔코더 값
    m2_speed = d_e2cnt * 60000/rotation;
    m2_speed /= (signed long)M2dt;// 거 = 속 시 RPM 값
    M2t_k_1 = M2t_k;//시간 초기화
    e2cnt_k_1 = e2cnt_k;//엔코더 초기화
  }
}



void M3RPM(int m3_ref_spd){
  
  M3t_k = millis();//시간측정 
  M3dt = M3t_k - M3t_k_1;//시간변화율
  if(M3dt >= 50){
    e3cnt_k = e3cnt; 
    d_e3cnt = e3cnt_k - e3cnt_k_1;// 엔코더 값
    m3_speed = d_e3cnt * 60000/rotation;
    m3_speed /= (signed long)M3dt;// 거 = 속 시 RPM 값
    M3t_k_1 = M3t_k;//시간 초기화
    e3cnt_k_1 = e3cnt_k;//엔코더 초기화
  }
}

void M4RPM(int m4_ref_spd){
  
  M4t_k = millis();//시간측정 
  M4dt = M4t_k - M4t_k_1;//시간변화율
  if(M4dt >= 50){
    e4cnt_k = e4cnt; 
    d_e4cnt = e4cnt_k - e4cnt_k_1;// 엔코더 값
    m4_speed = d_e4cnt * 60000/rotation;
    m4_speed /= (signed long)M4dt;// 거 = 속 시 RPM 값
    M4t_k_1 = M4t_k;//시간 초기화
    e4cnt_k_1 = e4cnt_k;//엔코더 초기화
  }
}

void M1doMotor(bool dir, long vel){
  
  digitalWrite(M1_I1, dir);
  digitalWrite(M1_I2, !dir);
  vel = abs(vel);
  m1_ipwm_u = vel > 255 ? 255 : vel; 
  analogWrite(M1_PWM, m1_ipwm_u);
}

void M2doMotor(bool dir, long vel){
  
  digitalWrite(M2_I1, dir);
  digitalWrite(M2_I2, !dir);
  vel = abs(vel);
  m2_ipwm_u = vel > 255 ? 255 : vel; 
  analogWrite(M2_PWM, m2_ipwm_u);
}
void M3doMotor(bool dir, long vel){
  
  digitalWrite(M3_I1, !dir);
  digitalWrite(M3_I2, dir);
  vel = abs(vel);
  m3_ipwm_u = vel > 255 ? 255 : vel; 
  analogWrite(M3_PWM, m3_ipwm_u);
}
void M4doMotor(bool dir, long vel){
  
  digitalWrite(M4_I1, !dir);
  digitalWrite(M4_I2, dir);
  vel = abs(vel);
  m4_ipwm_u = vel > 255 ? 255 : vel; 
  analogWrite(M4_PWM, m4_ipwm_u);

}


void Enc1chA_ISR(){  e1cnt += (digitalRead(ENC1_CHA)==digitalRead(ENC1_CHB))?1:-1;}
void Enc2chA_ISR(){  e2cnt += (digitalRead(ENC2_CHA)==digitalRead(ENC2_CHB))?1:-1;}
void Enc3chA_ISR(){  e4cnt += (digitalRead(2)==digitalRead(3))?1:-1;}
void Enc4chA_ISR(){  e4cnt += (digitalRead(2)==digitalRead(3))?1:-1;}
