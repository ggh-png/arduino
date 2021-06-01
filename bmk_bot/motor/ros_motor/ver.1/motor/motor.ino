#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <TimerOne.h>

#define LOOP_TIME        200000 

 
#include "PID.h"

#define RPM_to_RADIAN 3.141592654*2/60
#define RADIAN_to_RPM 60/(3.141592654*2)

#define WHEEL_RADIUS 0.068 // 바퀴 반지름 
#define ROBOT_RADIUS 0.16  // 로봇 반지름 = 바퀴 사이의 거리 / 2
#define PI 3.141592654



ros::NodeHandle  nh;

std_msgs::Float32 left_wheel_vel;

ros::Publisher left_wheel_vel_pub("/left_wheel_velocity", &left_wheel_vel);

std_msgs::Float32 right_wheel_vel;

ros::Publisher right_wheel_vel_pub("/right_wheel_velocity", &right_wheel_vel);

geometry_msgs::Twist sensor_vel;
ros::Publisher sensor_vel_pub("/sensor_velocity", &sensor_vel);



 void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  //Left Motor Speed
  M3RPM();
  M4RPM();
  
  left_wheel_vel.data = m3_speed * RPM_to_RADIAN;
  left_wheel_vel_pub.publish(&left_wheel_vel);
  
  right_wheel_vel.data = m2_speed * RPM_to_RADIAN;
  right_wheel_vel_pub.publish(&right_wheel_vel);
  //속도(m/s) V = (r * (W1 + w2))/2
  sensor_vel.linear.x = WHEEL_RADIUS*(left_wheel_vel.data + right_wheel_vel.data)/2;
  sensor_vel.linear.y = 0;
  sensor_vel.linear.z = 0;
  //각속도(rad/s) (V1 - V2) / L
  sensor_vel.angular.x = 0;
  sensor_vel.angular.y = 0;
  sensor_vel.angular.z = (WHEEL_RADIUS * (left_wheel_vel.data - right_wheel_vel.data)/2) / (ROBOT_RADIUS);
  sensor_vel_pub.publish(&sensor_vel);
  Timer1.attachInterrupt(timerIsr);  //enable the timer
}


 
void cmdLeftWheelCB( const std_msgs::Int16& msg)
{
  M3vel_PID(msg.data);
}

void cmdRightWheelCB( const std_msgs::Int16& msg)
{
  M2vel_PID(msg.data);
}



ros::Subscriber<std_msgs::Int16> subCmdLeft("cmd_left_wheel", cmdLeftWheelCB );
ros::Subscriber<std_msgs::Int16> subCmdRight("cmd_right_wheel",cmdRightWheelCB );





void cmdVelCB( const geometry_msgs::Twist& twist)
{

  double linear_left_RPM,linear_right_RPM;
  linear_left_RPM =linear_right_RPM= twist.linear.x/(2*PI*WHEEL_RADIUS)*60;

  double angular_left_RPM, angular_right_RPM;
  angular_right_RPM = twist.angular.z * (ROBOT_RADIUS/WHEEL_RADIUS) * 60 / 2 / PI;
  
  angular_left_RPM = -angular_right_RPM;

  double left_target_RPM = linear_left_RPM + angular_left_RPM;
  double right_target_RPM = linear_right_RPM + angular_right_RPM;



  
  M3vel_PID(left_target_RPM);
  M2vel_PID(right_target_RPM); 
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", cmdVelCB);




void setup() {
  // put your setup code here, to run once:


  pinMode(ENC1_CHA, INPUT_PULLUP);
  pinMode(ENC1_CHB, INPUT_PULLUP);
  pinMode(ENC2_CHA, INPUT_PULLUP);
  pinMode(ENC2_CHB, INPUT_PULLUP);
  pinMode(ENC3_CHA, INPUT_PULLUP);
  pinMode(ENC3_CHB, INPUT_PULLUP);
  pinMode(ENC4_CHA, INPUT_PULLUP);
  pinMode(ENC4_CHB, INPUT_PULLUP);

  Timer1.initialize(LOOP_TIME);

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
  Serial.begin(57600);


  nh.initNode();
  nh.subscribe(subCmdRight);
  nh.subscribe(subCmdLeft);
  nh.subscribe(subCmdVel);
  nh.advertise(left_wheel_vel_pub);
  nh.advertise(right_wheel_vel_pub);
  nh.advertise(sensor_vel_pub);
  
  Timer1.attachInterrupt( timerIsr ); // enable the timer
}

void loop() 
{   

    nh.spinOnce();  
}
