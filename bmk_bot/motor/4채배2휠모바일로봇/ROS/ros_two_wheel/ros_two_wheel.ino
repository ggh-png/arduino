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


float speed_1 = 0;
float speed_2 = 0; 
void cmdLeftWheelCB( const std_msgs::Float32& left_wheel_vel)
{
  speed_1 = left_wheel_vel.data;
}

void cmdRightWheelCB( const std_msgs::Float32& right_wheel_vel)
{
  speed_2 = right_wheel_vel.data;
}



 void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  //Left Motor Speed

  
  left_wheel_vel.data = m1_speed;
  left_wheel_vel_pub.publish(&left_wheel_vel);
  
  right_wheel_vel.data = m2_speed;
  right_wheel_vel_pub.publish(&right_wheel_vel);
  M1vel_PID(speed_1);
  M2vel_PID(speed_2); 
  Timer1.attachInterrupt(timerIsr);  //enable the timer
}





ros::Subscriber<std_msgs::Int16> subCmdLeft("cmd_left_wheel", cmdLeftWheelCB );
ros::Subscriber<std_msgs::Int16> subCmdRight("cmd_right_wheel",cmdRightWheelCB );







void setup() {
  // put your setup code here, to run once:


  pinMode(ENC1_CHA, INPUT_PULLUP);
  pinMode(ENC1_CHB, INPUT_PULLUP);
  pinMode(ENC2_CHA, INPUT_PULLUP);
  pinMode(ENC2_CHB, INPUT_PULLUP);


  Timer1.initialize(LOOP_TIME);

  Serial.begin(57600);


  nh.initNode();
  nh.subscribe(subCmdRight);
  nh.subscribe(subCmdLeft);
  nh.advertise(left_wheel_vel_pub);
  nh.advertise(right_wheel_vel_pub);

  
  Timer1.attachInterrupt( timerIsr ); // enable the timer
}

void loop() 
{   

    nh.spinOnce();  
}
