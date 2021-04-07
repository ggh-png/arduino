// motor control pin
#define motorDirPin  4 //
#define motorPWMPin  5 //

// encoder pin
#define encoderPinB  2
#define encoderPinA  3
#define rotation 4320


int encoderPos = 0;
const float ratio = 360./rotation;

// P control
float Kp = 10; // p - gain 
float Deg = 360; // 22의 값은 모터 자체의 백레쉬를 뜻한다.

// enc

void doEncoderA();
void doEncoderB();
void doMotor(bool dir, int vel);
void motor (int enc, int vel);


void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);
 
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderB, CHANGE);
 
  pinMode(motorDirPin, OUTPUT);

  Serial.begin(115200);
}

void loop() {

  doMotor(0, 40);
  int test = 0?10:(10 - 10);
  Serial.println(test);


}


void doEncoderA(){  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?1:-1;}
void doEncoderB(){  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?-1:1;}

void doMotor(bool dir, int vel){
  digitalWrite(motorDirPin, dir);
  analogWrite(motorPWMPin, dir?vel:(255 - vel));
}


void motor (int enc, int vel2){
 
  float motorDeg = encoderPos*ratio;
  float error = enc - motorDeg; // 목표 값 - 실제 값  
  float control = Kp * error;
  doMotor( (control>=0)?HIGH:LOW, min(abs(control), vel2));

  Serial.print("encoderPos : "); //현 enc position
  Serial.print(encoderPos);
  Serial.print("   motorDeg : "); //시작점 부터의 각도
  Serial.print(motorDeg);
  Serial.print("   error : "); 
  Serial.print(error);
  Serial.print("    control : ");
  Serial.print(control);
  Serial.print("    motorVel : ");
  Serial.println(min(abs(control), vel2));
}
