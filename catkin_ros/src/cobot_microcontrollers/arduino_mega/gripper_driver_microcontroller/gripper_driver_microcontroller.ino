//<description>gripper driver arduino mega script for catkin_ros workspace, April 12, 2023</description>
//<maintainer email="jcheng2020@my.fit.edu">Junfu Cheng</maintainer>
//<author email="jcheng2020@my.fit.edu">Junfu Cheng</author>
#include "vexServo.hpp"

//const int controlPin = 7;
int vexServoPin = 9;
vexServo motor;
enum StateValue{ close, open} state;
// Firmware version
const char* VERSION = "0.0.1";

//jcheng2020 revised angle to distance 90/0.032 = 2812.5 , distance 0.020 m is the cirtial point hence 56.25 is the critical angle
const float criticalTarget= 56.25/*34*/;
long long time = millis();
long long instant;
bool run = false;

String inData = "";
String msg;
String function;

void control( bool command);
void generateAPulse();
void stateTRAJ();

void setup() {
  // put your setup code here, to run once:
  //pinMode( controlPin, OUTPUT);
  state = close;
  motor.initial(vexServoPin);
  Serial.begin(115200);

  motor.setPosition(0);
  time = millis();
  while(millis() - time < 1000)
  {
    motor.run();
  }
  motor.setPosition(6);
  time = millis();
    while(millis() - time < 1000)
  {
    motor.run();
  }
  motor.setPosition(0);
  time = millis();
  while(millis() - time < 1000)
  {
    motor.run();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  // clear message
  inData = "";
  while(true)
  {
    stateTRAJ();
  }
}

void control( StateValue command)
{
   if(command == open)
   {
     motor.setPosition(6);
     motor.run();
   }
   if(command == close)
   {
     motor.setPosition(0);
     motor.run();
   }


//  instant = millis() - time;
//  if( instant > 500)
//  {
//    if(state != command)
//    {
//        run = true;
//        state = command;
//    }
//    else
//    {
//      run = false;
//    }
//    time = millis();
//  }
//  if( run == true)
//  {
//    generateAPulse();
//  }
}
/*
void generateAPulse()
{
    instant = millis() - time;
    if( instant < 100)
    {
      digitalWrite( controlPin, LOW);
    }
    if( instant >= 100 && instant < 400)
    {
      digitalWrite( controlPin, HIGH);
    }
    if( instant >= 400 && instant < 500)
    {
      digitalWrite( controlPin, LOW);
      run = false;
    }
}
*/
void stateTRAJ()
{
  char received = "";
  
  if (Serial.available())
  {
    received = Serial.read();
    inData += received;
  }
  // process message when new line character is received
  if (received == '\n')
  {
    function = inData.substring(0, 2);
    // update trajectory information
    if (function == "MT")
    {
      int msgIdxA = inData.indexOf('A');
      int msgIdxB = inData.indexOf('B');
      int target = inData.substring(msgIdxA + 1, msgIdxB).toInt();
      //distance 0.012 m is the cirtial point hence 34 is the critical angle
      if(target < criticalTarget)
      {
        control( close);
        msg = String("JP") + String( "close") + String( "\n");
      }
      else
      {
        control( open);
        msg = String("JP") + String( "open_") + String( "\n");
      }
      Serial.print(msg);
    }

    
    else if ( function == "ST")
    {          
      int idxVersion = inData.indexOf('A');
      String softwareVersion = inData.substring(idxVersion + 1, inData.length() - 1);
      if( softwareVersion == VERSION)
      {
        msg = String("ST") + String("A") + String( 1) + String("B") + String(VERSION) + String("\n");
      }
      else
      {
        msg = String("ST") + String("A") + String( 0) + String("B") + String(VERSION) + String("\n");
      }
      Serial.print(msg);
    }

    
    else if (function == "JC")
    {
      control( close);
      state = close;
      msg = String( "JC") + String( "\n");
      Serial.print( msg);
    }

    
    else if(function == "JP")
    {
      if( state == close )
      {
        msg = String( "JP") + String( "close") + String( "\n");
      }
      else if( state == open)
      {
        msg = String( "JP") + String( "open_") + String( "\n");
      }
      Serial.print( msg);
    }
    inData = "";
  }
}
