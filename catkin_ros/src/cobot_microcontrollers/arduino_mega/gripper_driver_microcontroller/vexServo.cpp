/*
  *Florida Institute of Technology
  *Author: Junfu Cheng, jcheng2020@my.fit.edu
  *Course: ECE 4241, Section 01, Fall 2022
  *vexServo.cpp is a cpp file of Arduino Uno sketch for a gripper drived by a Vex 2-Wire Motor 393 with Vex Motor Controller 29 (7.2 V power supply required)
  *For Vex Motor Controller 29, the voltage from the orange pin to the black pin should be around 7.2 V and the white pin is for a control signal
*/

#include "vexServo.hpp"
const int maxSpeedValue = 300;//The gripper releases
const int minSpeedValue = -300;//The gripper catches
const int maxTempSpeedValue = 1800;//The gripper releases
const int minTempSpeedValue = 1200;//The gripper catches
const int maxPositionValue = 6;//The release state of the gripper
const int minPositionValue = 0;//The catch state of the gripper

int vexServo::initial(int pin)
{
  pinMode(pin, OUTPUT);
  attach(pin);
  return pin;
}

void vexServo::speed(int speedValue)
{
  if(speedValue > maxSpeedValue)
  {
    speedValue = maxSpeedValue;
  }
  else if(speedValue < minSpeedValue)
  {
    speedValue = minSpeedValue;
  }

  write(map(speedValue,minSpeedValue,maxSpeedValue,minTempSpeedValue,maxTempSpeedValue));
}

void vexServo::setPosition(int targetPositionValue)
{
  this->targetPositionValue = targetPositionValue;
  if(targetPositionValue > maxPositionValue)
  {
    this->targetPositionValue = maxPositionValue;
  }
  else if(targetPositionValue < minPositionValue)
  {
    this->targetPositionValue = minPositionValue;
  }
  

}

void vexServo::run()
{
    if(targetPositionValue - position > 0)//release direction
    {

        step(true);
        position++;

    }
    else if(targetPositionValue - position < 0)//catch direction
    {

        step(false);
        position--;

    }
}

void vexServo::step(bool direction)
{
  if(direction == true)
  {
    long time = millis();
    while(millis()-time<120)
    {
      speed(300);
    }
    speed(0);
  }
  else
  {
    long time = millis();
    while(millis()-time<30)
    {
      speed(-300);
    }
    speed(0);
  }
}
