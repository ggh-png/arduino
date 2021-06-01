
void zero_pose();
void pose_1();
void pose_2();
void pose_3();
void pose_4();
void pose_5();
void pose_6();


void keyboard_function(){
  if(Serial.available()) {
  preMode = newMode;
  newMode = Serial.read();
  }
  switch(newMode) {
  case '0':
  zero_pose();  
  break;
 
  case '1':
  pose_1();
  break;
 
  case '2':
  pose_2();
  break;
 
  case '3':
  pose_3();
  break;
   
  case '4':
  pose_4();
  break;
   
  case '5':
  pose_5();
  break;
   
  case '6':
  pose_6();
  break;
  }
  case 'w':
  pose_6();
  break;
  }
  case 'a':
  pose_6();
  break;
  }
  case 's':
  pose_6();
  break;
  }
  case 'd':
  pose_6();
  break;
  }
  case 'f':
  pose_6();
  break;
  }
  case 'x':
  pose_6();
  break;
  }   
}

void zero_pose(){
    LobotSerialServoMove(Serial3, ID1,Servo_To_Degrees(120), 1000);
    LobotSerialServoMove(Serial3, ID2,Servo_To_Degrees(120), 1000);
    LobotSerialServoMove(Serial3, ID3,Servo_To_Degrees(120), 1000);    
    LobotSerialServoMove(Serial3, ID4,Servo_To_Degrees(48), 1000);
}



void pose_1(){
    LobotSerialServoMove(Serial3, ID1,Servo_To_Degrees(120), 1000);
    LobotSerialServoMove(Serial3, ID2,Servo_To_Degrees(120), 1000);
    LobotSerialServoMove(Serial3, ID3,Servo_To_Degrees(120), 1000);    
    LobotSerialServoMove(Serial3, ID4,Servo_To_Degrees(192), 1000);
}



void pose_2(){
    LobotSerialServoMove(Serial3, ID1,Servo_To_Degrees(120), 1000);
    LobotSerialServoMove(Serial3, ID2,Servo_To_Degrees(30), 1000);
    LobotSerialServoMove(Serial3, ID3,Servo_To_Degrees(30), 1000);    
    LobotSerialServoMove(Serial3, ID4,Servo_To_Degrees(48), 1000);
}



void pose_3(){
    LobotSerialServoMove(Serial3, ID1,Servo_To_Degrees(120), 1000);
    LobotSerialServoMove(Serial3, ID2,Servo_To_Degrees(30), 1000);
    LobotSerialServoMove(Serial3, ID3,Servo_To_Degrees(30), 1000);    
    LobotSerialServoMove(Serial3, ID4,Servo_To_Degrees(192), 1000);
}


void pose_4(){
    LobotSerialServoMove(Serial3, ID1,Servo_To_Degrees(120), 1000);
    LobotSerialServoMove(Serial3, ID2,Servo_To_Degrees(30), 1000);
    LobotSerialServoMove(Serial3, ID3,Servo_To_Degrees(120), 1000);    
    LobotSerialServoMove(Serial3, ID4,Servo_To_Degrees(48), 1000);
}


void pose_5(){
    LobotSerialServoMove(Serial3, ID1,Servo_To_Degrees(120), 1000);
    LobotSerialServoMove(Serial3, ID2,Servo_To_Degrees(30), 1000);
    LobotSerialServoMove(Serial3, ID3,Servo_To_Degrees(120), 1000);    
    LobotSerialServoMove(Serial3, ID4,Servo_To_Degrees(192), 1000);
}


void pose_6(){
    LobotSerialServoMove(Serial3, ID1,Servo_To_Degrees(90), 1000);
    LobotSerialServoMove(Serial3, ID2,Servo_To_Degrees(30), 1000);
    LobotSerialServoMove(Serial3, ID3,Servo_To_Degrees(120), 1000);    
    LobotSerialServoMove(Serial3, ID4,Servo_To_Degrees(192), 1000);
}


double Servo_To_Degrees(float position_radians)
{
  return position_radians / 0.24;
}

  
