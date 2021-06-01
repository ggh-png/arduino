/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include "servo.h" 
#include <ros.h>
#include <std_msgs/UInt16.h>


#define ID3   3

ros::NodeHandle  nh;


void servo_cb( const std_msgs::UInt16& cmd_msg){          //cmd 명령어를 쓰겠다.
 
  LobotSerialServoMove(Serial3, 4, cmd_msg.data, 1000);
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led 
}


ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);//  servo_cb 함수를 sub으로 지정 

void setup(){
  Serial3.begin(115200); 
  pinMode(13, OUTPUT);
  nh.initNode(); // 노드명 초기화
  nh.subscribe(sub); //sub 으로 지정 
  LobotSerialServoSetMode(Serial3, 4, 500, 1000);  
}

void loop(){
  nh.spinOnce();
  delay(1);
}
