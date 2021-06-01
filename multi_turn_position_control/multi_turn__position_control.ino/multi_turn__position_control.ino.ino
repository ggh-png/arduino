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
float Kp = 10;
float targetDeg = 360*3.5 + 22;

void doEncoderA();
void doEncoderB();
void doMotor(bool dir, int vel);



void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);
 
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderB, CHANGE);
 
  pinMode(motorDirPin, OUTPUT);

  Serial.begin(115200);
}

void loop() {

  motor(3600, 255);

}


void doEncoderA(){  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?1:1;}
//void doEncoderB(){  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?-1:1;}

void doMotor(bool dir, int vel){
  digitalWrite(motorDirPin, dir);
  analogWrite(motorPWMPin, dir?vel:vel);
}
void motor (int enc, int vel){

  float motorDeg = encoderPos*ratio;
 
  float error = enc - motorDeg;
  float control = Kp*error;

  doMotor( (control>=0)?HIGH:LOW, min(abs(control), vel));

  Serial.print("encoderPos : ");
  Serial.print(encoderPos);
  Serial.print("   motorDeg : ");
  Serial.print(motorDeg);
  Serial.print("   error : ");
    Serial.print(error);
  Serial.print("    control : ");
  Serial.print(control);
  Serial.print("    motorVel : ");
  Serial.println(min(abs(control), 255));  
}
