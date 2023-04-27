/*
  *Florida Institute of Technology
  *Author: Junfu Cheng, jcheng2020@my.fit.edu
  *Course: ECE 4241, Section 01, Fall 2022
  *vexServo.hpp is a head file of Arduino Uno sketch for a gripper drived by a Vex 2-Wire Motor 393 with Vex Motor Controller 29 (7.2 V power supply required)
  *For Vex Motor Controller 29, the voltage from the orange pin to the black pin should be around 7.2 V and the white pin is for a control signal
*/

#ifndef vexServo_hpp
#define vexServo_hpp

#include "Arduino.h"
#include "Servo.h"
class vexServo: public Servo
{
  public:
    int initial(int pin);
    //for set target pin, initialize the vexServo type variable by the member function vexServo::initial(int pin) firstly, 
    //then, member function vexServo::speed(int speedValue) and vexServo::setPosition(int positionValue) are available
    //setting the gripper to catch state manually is required initally.
    void speed(int speedValue);
    //the range of int variable speedValue should be in the interval [-300,300],
    //if speedValue > 0, the gripper moves in release direction
    //if speedValue < 0, the gripper moves in catches direction
    //if speedValue == 0, the gripper remains static
    void setPosition(int positionValue);
    //the range of position should be in the interval [0,6],
    //where 0 is the catch state of the gripper
    //where 6 is the release state of the gripper
    void run();
  private:
    void step(bool direction);
    int position = 0;
    long time;
    int targetPositionValue;
};
#endif
