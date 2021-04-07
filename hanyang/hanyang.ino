#define ENC1_CHA 2 //Yellow
#define ENC1_CHB 3 //Green
#define M1_DIR 4
#define M1_PWM 5
#define Kp 16
unsigned long t_k = 0, t_k_1 = 0, dt = 0;
int e1cnt = 0;
int e1cnt_k = 0;
int e1cnt_k_1 = 0;
int d_e1cnt = 0;
long m1speed = 0;
int m1_ref_spd = 0;
long m1_err_spd = 0;
long ctrl_up = 0;
int ipwm_u = 0;


void setup(){
pinMode(ENC1_CHA, INPUT_PULLUP);
pinMode(ENC1_CHB, INPUT_PULLUP);
pinMode(M1_DIR, OUTPUT);
pinMode(M1_PWM, OUTPUT);
attachInterrupt(digitalPinToInterrupt(ENC1_CHA), Enc1chA_ISR, CHANGE);
attachInterrupt(digitalPinToInterrupt(ENC1_CHB), Enc1chB_ISR, CHANGE);
Serial.begin(115200);
digitalWrite(M1_DIR, LOW); //LOW --> CCW
analogWrite(M1_PWM, 0);
}

void loop(){
if(Serial.available()){
m1_ref_spd = Serial.parseInt();
}
t_k = millis();
dt = t_k - t_k_1;
if(dt >= 50){
e1cnt_k = e1cnt;
d_e1cnt = e1cnt_k - e1cnt_k_1;
m1speed = d_e1cnt * 15.15;
m1speed /= (signed long)dt;
t_k_1 = t_k;
e1cnt_k_1 = e1cnt_k;
//Error
m1_err_spd = m1_ref_spd - m1speed;
//P-Controller
ctrl_up = Kp * m1_err_spd;

if(ctrl_up >= 0){
digitalWrite(M1_DIR, LOW); //ccw
if(ctrl_up > 255) ipwm_u = 255;
else ipwm_u = (int)ctrl_up;
}
else{
digitalWrite(M1_DIR, HIGH); //cw
if(ctrl_up < -255) ipwm_u = 255;
else ipwm_u = (int)ctrl_up * (-1);
}
analogWrite(M1_PWM, ipwm_u);
Serial.print(m1speed);
Serial.print(",");
Serial.println(ctrl_up);
}
}
