//<description>An Arduino script for receiving sensor reading from the sensor chessboard to detect the positions of chess pieces and transmitting the data to ROS subscribers</description>
//<maintainer email="jcheng2020@my.fit.edu">Junfu Cheng</maintainer>
//<author email="jcheng2020@my.fit.edu">Junfu Cheng</author>

#include <ros.h>
#include <std_msgs/String.h>
#include "Button.hpp"

int ButtonPin = 13;

Button control(ButtonPin);



//variables and constants for sensor chessboard
const int COLUMN_NUMBER = 8;
const int ROW_NUMBER = 8;
const int inputPin[ROW_NUMBER] = { 4, 5, 6, 7, 8, 9, 10, 11};
/*
const float threshold[ROW_NUMBER][COLUMN_NUMBER] = 
{{0.60, 0.55, 0.45, 0.73, 0.51, 0.84, 0.89, 0.96},
 {0.95, 1.60, 1.70, 1.10, 1.32, 1.24, 1.29, 0.78},
 {0.69, 1.80, 1.74, 1.97, 1.64, 1.00, 1.18, 1.25},
 {0.86, 1.90, 1.90, 2.05, 2.30, 1.21, 2.18, 1.23},
 {1.50, 1.89, 1.42, 1.76, 1.46, 1.06, 0.95, 1.40},
 {0.57, 1.24, 1.19, 1.16, 0.89, 1.51, 1.09, 1.30},
 {0.77, 0.45, 0.74, 0.85, 0.62, 0.69, 1.29, 0.89},
 {0.60, 0.56, 0.54, 0.78, 0.55, 0.44, 0.52, 0.74}};
 */
 const float threshold[ROW_NUMBER][COLUMN_NUMBER] = 
{{0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20},
 {0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.40, 0.20},
 {0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20},
 {0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20},
 {0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20},
 {0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20},
 {0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20},
 {0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20}};
//float output[ROW_NUMBER][COLUMN_NUMBER];
bool output[ROW_NUMBER][COLUMN_NUMBER];// false is empty, true is occupied
float voltage[COLUMN_NUMBER];
char display[49];
char temp;

//initialize digital inputs of the sensor chessboard
void chessboard_setup()
{
  for(int i = 0; i < ROW_NUMBER;i ++)
  {
    pinMode( inputPin[i], OUTPUT);
    digitalWrite( inputPin[i], HIGH);
  }
}






//varibales for ros transmission
ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("msgs_from_sensor_chessboard", &str_msg);
char transmission[49];

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(chatter);
  chessboard_setup();
}

void loop() {
  // put your main code here, to run repeatedly:

  //detect all sensor reading
  for(int i = 0; i < ROW_NUMBER; i++)
  {
    //activate the input corresponding to each ROW for each ROW detection
    digitalWrite( inputPin[i], LOW);

    //read the input for each COLUMN in a ROW
    voltage[0] = analogRead(A0) * (5.0 / 1023.0);
    voltage[1] = analogRead(A1) * (5.0 / 1023.0);
    voltage[2] = analogRead(A2) * (5.0 / 1023.0);
    voltage[3] = analogRead(A3) * (5.0 / 1023.0);
    voltage[4] = analogRead(A4) * (5.0 / 1023.0);
    voltage[5] = analogRead(A5) * (5.0 / 1023.0);
    voltage[6] = analogRead(A6) * (5.0 / 1023.0);
    voltage[7] = analogRead(A7) * (5.0 / 1023.0);

    //save reading in the output array
    for(int j = 0; j < COLUMN_NUMBER; j++)
    {
      //output[i][j] = voltage[j];
      
      if(voltage[j] < threshold[i][j])
      {
        output[i][j] = true;
      }
      else
      {
        output[i][j] = false;
      }
      
    }

    //deactivate the input corresponding to each ROW
    digitalWrite( inputPin[i], HIGH);
  }

  //broadcast Instruction
  display[0] = 'i';
  if(control.update() == Submit)
  {
    display[1] = 'S';
  }
  else
  {
    display[1] = 'W';
  }
  display[2] = '\0';
  for(int i = 0 ; i < 49; i++)
  {
    transmission[i] = display[i];
  }
  //transmit data in ROS
  str_msg.data = transmission;
  chatter.publish( &str_msg );
  nh.spinOnce();


  //construct the display String
  temp = '1';
  for(int i = 0; i < ROW_NUMBER; i++)
  {
    display[0] = temp;
    for(int j = 0; j < COLUMN_NUMBER; j++)
    {
      /*
      String temp = "";
      temp = temp + output[i][j];
      display[1 + j * 5] = temp[0];
      display[1 + j * 5 + 1] = temp[1];
      display[1 + j * 5 + 2] = temp[2];
      display[1 + j * 5 + 3] = temp[3];
      display[1 + j * 5 + 4] = '|';
      */
      
      if(output[i][j] == true)
      {
        display[j*2+1] = '1';
      }
      else
      {
        display[j*2+1] ='0';
      }
      display[j*2 + 2] ='|';
      
    }
    display[COLUMN_NUMBER * 3 + 1]='\0';
    for(int i = 0 ; i < 49; i++)
    {
      transmission[i] = display[i];
    }
    //transmit data in ROS
    str_msg.data = transmission;
    chatter.publish( &str_msg );
    nh.spinOnce();
    temp = temp + 1;
  }
  
  delay(200);

}
