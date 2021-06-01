#include "PID.h"
byte newMode; // f:전진, b:후진, l:좌회전, r:우회전, s:정지, a:가속, d:감속
byte preMode;


//Robot parameter
#define L       45      //45mm
#define D       85      //85mm
#define LPLUSD  0.13    //L+D=130mm/1000 = 0.13m
#define R       30      //30mm



int b; 
void setup() {
  motor_setting();
}

void loop() {
  keyboard_function(); 
}




void MecanumVelocity(float vx, float vy, float dpi) {
  //vx, vy(robot velocity) unit -> cm/sec
  //dpi(robot angular velocity) unit -> deg/sec
  float v1, v2, v3, v4;
  float drot;

  //[deg/sec] to [rad/sec] :( *pi/180)--> cm(rad)/sec
//  dpi *= 31.41593;
//  dpi /= 18;

//  drot = LPLUSD * dpi;
  m1_speed = vx - vy - drot;
  m2_speed = vx + vy - drot;
  m3_speed = vx - vy + drot;
  m4_speed = vx + vy + drot;

  //wheel linear velocity to wheel speed
  m1_ref_spd = m1_speed * 8 / 3.141593;
  m2_ref_spd = m1_speed * 8 / 3.141593;
  m3_ref_spd = m1_speed * 8 / 3.141593;
  m4_ref_spd = m1_speed * 8 / 3.141593;
}





void keyboard_function(){
  if(Serial.available()) {
  preMode = newMode;
  newMode = Serial.read();
  }
 
  switch(newMode) {
  case 'w':
  M1vel_PID(15);
  M2vel_PID(15);
  M3vel_PID(-15);
  M4vel_PID(-15);  
  break;
 
  case 's':
  M1vel_PID(0);
  M2vel_PID(0);
  M3vel_PID(0);
  M4vel_PID(0);
  break;
 
  case 'a':
  M1vel_PID(15);
  M2vel_PID(-15);
  M3vel_PID(15);
  M4vel_PID(-15);
  break;
 
  case 'd':
  M1vel_PID(-15);
  M2vel_PID(15);
  M3vel_PID(-15);
  M4vel_PID(15);
  break;

  case 'x':
  M1vel_PID(-15);
  M2vel_PID(-15);
  M3vel_PID(15);
  M4vel_PID(15); 
  break;
  }
}
