/*  ARCS - Stepper motor robot control software Ver 1.0 jcheng2020 revised version with new calibration procedure and rosnode publishers/subscribers
    Copyright (c) 2019, Chris Annin
    All rights reserved.

    You are free to share, copy and redistribute in any medium
    or format.  You are free to remix, transform and build upon
    this material.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

          Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
          Redistribution of this software in source or binary forms shall be free
          of all charges or fees to the recipient of this software.
          Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
          you must give appropriate credit and indicate if changes were made. You may do
          so in any reasonable manner, but not in any way that suggests the
          licensor endorses you or your use.
          Selling AR2 software, robots, robot parts, or any versions of robots or software based on this
          work is strictly prohibited.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL CHRIS ANNIN BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    chris.annin@gmail.com

    Log:

*/

#include <ros.h>
#include "std_msgs/String.h"

#include <Encoder.h>
#include <avr/pgmspace.h>

#include "AccelStepper.h"


// Firmware version
const char* VERSION = "0.0.1";

// SPEED // millisecond multiplier // raise value to slow robot speeds // DEFAULT = 220
const int SpeedMult = 220;

/*
  MOTOR DIRECTION - motor directions can be changed on the caibration page in the software but can also
  be changed here: example using DM542T driver(CW) set to 1 - if using ST6600 or DM320T driver(CCW) set to 0
  DEFAULT = 111011   */

const int J1rotdir = 0;
const int J2rotdir = 1;
const int J3rotdir = 1;
const int J4rotdir = 0;
const int J5rotdir = 1;
const int J6rotdir = 1;
const int TRACKrotdir = 0;
const int ROT_DIRS[] = { J1rotdir, J2rotdir, J3rotdir, J4rotdir, J5rotdir, J6rotdir };

/* start positions - these are the joint step values at power up, default is in the rest position using
  the following values: J1=7600, J2=2322, J3=0, J4=7600, J5=2287, J6=3312 */

int J1startSteps = 7600;
int J2startSteps = 2322;
int J3startSteps = 0;
int J4startSteps = 7600;
int J5startSteps = 2287;
int J6startSteps = 3312;

// approx encoder counts at rest position, 0 degree joint angle
const int REST_ENC_POSITIONS[] = { 38681, 11264, 0, 36652, 5839, 15671 };

String inData;
String function;
char WayPt[200][70];
int WayPtDel;

const int J1stepPin = 0;
const int J1dirPin = 1;
const int J2stepPin = 2;
const int J2dirPin = 3;
const int J3stepPin = 4;
const int J3dirPin = 5;
const int J4stepPin = 6;
const int J4dirPin = 7;
const int J5stepPin = 8;
const int J5dirPin = 9;
const int J6stepPin = 10;
const int J6dirPin = 11;
const int TRstepPin = 12;
const int TRdirPin = 13;
const int STEP_PINS[] = { J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin };
const int DIR_PINS[] = { J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin };

//set encoder pins
Encoder J1encPos(14, 15);
Encoder J2encPos(16, 17);
Encoder J3encPos(18, 19);
Encoder J4encPos(20, 21);
Encoder J5encPos(22, 23);
Encoder J6encPos(24, 25);
Encoder JOINT_ENCODERS[] = { J1encPos, J2encPos, J3encPos, J4encPos, J5encPos, J6encPos };
int ENC_DIR[] = { 1, 1, 1, 1, 1, 1 }; // +1 if encoder direction matches motor direction

//set calibration limit switch pins
const int J1calPin = 26;
const int J2calPin = 27;
const int J3calPin = 28;
const int J4calPin = 29;
const int J5calPin = 30;
const int J6calPin = 31;
const int CAL_PINS[] = { J1calPin, J2calPin, J3calPin, J4calPin, J5calPin, J6calPin };
const int LIMIT_SWITCH_HIGH[] = { 1, 1, 1, 1, 1, 1 }; // to account for both NC and NO limit switches
const int CAL_DIR[] = { -1, -1, 1, -1, -1, 1 }; // joint rotation direction to limit switch
const int CAL_SPEED = 600; // motor steps per second
const int CAL_SPEED_MULT[] = { 1, 1, 1, 2, 1, 1 }; // multiplier to account for motor steps/rev

// motor and encoder steps per revolution
const int MOTOR_STEPS_PER_REV[] = { 400, 400, 400, 400, 800, 400 };

// num of steps in range of motion of joint, may vary depending on your limit switch
// defaults 77363, 36854, 40878, 71967, 23347, 32358
const int ENC_RANGE_STEPS[] = { 77363, 36854, 40878, 71967, 23347, 32358 };

//set encoder multiplier, encoder steps per motor step
const float J1encMult = 5.12;
const float J2encMult = 5.12;
const float J3encMult = 5.12;
const float J4encMult = 5.12;
const float J5encMult = 2.56;
const float J6encMult = 5.12;
const float EncDiv = .1;
const float ENC_MULT[] = { 5.12, 5.12, 5.12, 5.12, 2.56, 5.12 };

const int NUM_JOINTS = 6;
const float ENC_STEPS_PER_DEG[] = { 227.5555555555556, 284.4444444444444, 284.4444444444444, 223.0044444444444, 56.04224675948152, 108.0888888888889 };

// speed and acceleration settings
float JOINT_MAX_SPEED[] = { 20.0, 20.0, 20.0, 20.0, 20.0, 20.0 }; // deg/s
float JOINT_MAX_ACCEL[] = { 5.0, 5.0, 5.0, 5.0, 5.0, 5.0 }; // deg/s^2
int MOTOR_MAX_SPEED[] = { 1500, 15000, 1500, 2000, 1500, 1500 }; // motor steps per sec
int MOTOR_MAX_ACCEL[] = { 250, 2500, 250, 250, 250, 250 }; // motor steps per sec^2
float MOTOR_ACCEL_MULT[] = { 1.0, 1.0, 2.0, 1.0, 1.0, 1.0 }; // for tuning position control

AccelStepper stepperJoints[NUM_JOINTS];

enum SM { STATE_ARCS, STATE_TRAJ, STATE_ERR };
SM STATE = STATE_TRAJ; // default to STATE_TRAJ state


bool whetherReceive = false;
std_msgs::String str_msg;
ros::NodeHandle nh;
void messageCb(const std_msgs::String& msg){
  inData = msg.data;
  whetherReceive = true;
}
ros::Subscriber<std_msgs::String> sub("teensyDriver", &messageCb );
ros::Publisher chatter("teensyResponse", &str_msg);




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // run once:
  Serial.begin(115200);


  J1encPos.write(J1startSteps * J1encMult);
  J2encPos.write(J2startSteps * J2encMult);
  J3encPos.write(J3startSteps * J3encMult);
  J4encPos.write(J4startSteps * J4encMult);
  J5encPos.write(J5startSteps * J5encMult);
  J6encPos.write(J6startSteps * J6encMult);

  pinMode(TRstepPin, OUTPUT);
  pinMode(TRdirPin, OUTPUT);
  pinMode(J1stepPin, OUTPUT);
  pinMode(J1dirPin, OUTPUT);
  pinMode(J2stepPin, OUTPUT);
  pinMode(J2dirPin, OUTPUT);
  pinMode(J3stepPin, OUTPUT);
  pinMode(J3dirPin, OUTPUT);
  pinMode(J4stepPin, OUTPUT);
  pinMode(J4dirPin, OUTPUT);
  pinMode(J5stepPin, OUTPUT);
  pinMode(J5dirPin, OUTPUT);
  pinMode(J6stepPin, OUTPUT);
  pinMode(J6dirPin, OUTPUT);

  pinMode(J1calPin, INPUT_PULLUP);
  pinMode(J2calPin, INPUT_PULLUP);
  pinMode(J3calPin, INPUT_PULLUP);
  pinMode(J4calPin, INPUT_PULLUP);
  pinMode(J5calPin, INPUT_PULLUP);
  pinMode(J6calPin, INPUT_PULLUP);

  digitalWrite(TRstepPin, HIGH);
  digitalWrite(J1stepPin, HIGH);
  digitalWrite(J2stepPin, HIGH);
  digitalWrite(J3stepPin, HIGH);
  digitalWrite(J4stepPin, HIGH);
  digitalWrite(J5stepPin, HIGH);
  digitalWrite(J6stepPin, HIGH);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);

}


  

bool initStateTraj(String inData)
{
  // parse initialisation message
  int idxVersion = inData.indexOf('A');
  String softwareVersion = inData.substring(idxVersion + 1, inData.length() - 1);
  int versionMatches = (softwareVersion == VERSION);

  // return acknowledgement with result
  String msg = String("ST") + String("A") + String(versionMatches) + String("B") + String(VERSION) + String("\n");
  //Serial.print(msg);
 
  str_msg.data = msg.c_str();
  chatter.publish( &str_msg );
  nh.spinOnce();

  return versionMatches ? true : false;
}

void readEncPos(int* encPos)
{
  encPos[0] = J1encPos.read() * ENC_DIR[0];
  encPos[1] = J2encPos.read() * ENC_DIR[1];
  encPos[2] = J3encPos.read() * ENC_DIR[2];
  encPos[3] = J4encPos.read() * ENC_DIR[3];
  encPos[4] = J5encPos.read() * ENC_DIR[4];
  encPos[5] = J6encPos.read() * ENC_DIR[5];

  //jcheng2020 disable encoders revise
  encPos[0] = stepperJoints[0].currentPosition() * ENC_MULT[0] * ENC_DIR[0];
  encPos[1] = stepperJoints[1].currentPosition() * ENC_MULT[1] * ENC_DIR[1];
  encPos[2] = stepperJoints[2].currentPosition() * ENC_MULT[2] * ENC_DIR[2];
  encPos[3] = stepperJoints[3].currentPosition() * ENC_MULT[3] * ENC_DIR[3];
  encPos[4] = stepperJoints[4].currentPosition() * ENC_MULT[4] * ENC_DIR[4];
  encPos[5] = stepperJoints[5].currentPosition() * ENC_MULT[5] * ENC_DIR[5];
  //jcheng2020 disable encoders revise end
}

void updateStepperSpeed(String inData)
{
  int idxSpeedJ1 = inData.indexOf('A');
  int idxAccelJ1 = inData.indexOf('B');
  int idxSpeedJ2 = inData.indexOf('C');
  int idxAccelJ2 = inData.indexOf('D');
  int idxSpeedJ3 = inData.indexOf('E');
  int idxAccelJ3 = inData.indexOf('F');
  int idxSpeedJ4 = inData.indexOf('G');
  int idxAccelJ4 = inData.indexOf('H');
  int idxSpeedJ5 = inData.indexOf('I');
  int idxAccelJ5 = inData.indexOf('J');
  int idxSpeedJ6 = inData.indexOf('K');
  int idxAccelJ6 = inData.indexOf('L');

  JOINT_MAX_SPEED[0] = inData.substring(idxSpeedJ1 + 1, idxAccelJ1).toFloat();
  JOINT_MAX_ACCEL[0] = inData.substring(idxAccelJ1 + 1, idxSpeedJ2).toFloat();
  JOINT_MAX_SPEED[1] = inData.substring(idxSpeedJ2 + 1, idxAccelJ2).toFloat();
  JOINT_MAX_ACCEL[1] = inData.substring(idxAccelJ2 + 1, idxSpeedJ3).toFloat();
  JOINT_MAX_SPEED[2] = inData.substring(idxSpeedJ3 + 1, idxAccelJ3).toFloat();
  JOINT_MAX_ACCEL[2] = inData.substring(idxAccelJ3 + 1, idxSpeedJ4).toFloat();
  JOINT_MAX_SPEED[3] = inData.substring(idxSpeedJ4 + 1, idxAccelJ4).toFloat();
  JOINT_MAX_ACCEL[3] = inData.substring(idxAccelJ4 + 1, idxSpeedJ5).toFloat();
  JOINT_MAX_SPEED[4] = inData.substring(idxSpeedJ5 + 1, idxAccelJ5).toFloat();
  JOINT_MAX_ACCEL[4] = inData.substring(idxAccelJ5 + 1, idxSpeedJ6).toFloat();
  JOINT_MAX_SPEED[5] = inData.substring(idxSpeedJ6 + 1, idxAccelJ6).toFloat();
  JOINT_MAX_ACCEL[5] = inData.substring(idxAccelJ6 + 1).toFloat();
}


//jcheng2020 revised begin
void calRetrieveTemp(int i)
{
  int curEncSteps[NUM_JOINTS];
  bool posReached;
  int tempRetrieveTarget;

  if(i == 0)
  {
    J1encPos.write(0);
  }
  else if(i == 1)
  {
    J2encPos.write(0);
  }
  else if(i == 2)
  {
    J3encPos.write(ENC_RANGE_STEPS[2]);
  }
  else if(i == 3)
  {
    J4encPos.write(0);
  }
  else if(i == 4)
  {
    J5encPos.write(0);
  }
  else if(i == 5)
  {
    J6encPos.write(ENC_RANGE_STEPS[5]);
  }
  readEncPos(curEncSteps);
  
  tempRetrieveTarget = (REST_ENC_POSITIONS[i] - curEncSteps[i])/10 + curEncSteps[i];

  
  stepperJoints[i].move((tempRetrieveTarget - curEncSteps[i])/ ENC_MULT[i]);
  posReached = false;
  while (!posReached)
  {
    posReached = true;
    readEncPos(curEncSteps);
    if (abs(tempRetrieveTarget - curEncSteps[i]) > 5)
    {
      posReached = false;
      stepperJoints[i].move((tempRetrieveTarget - curEncSteps[i]) / ENC_MULT[i]);
      //stepperJoints[i].setSpeed(-CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i]);
      stepperJoints[i].run();
    }
  }
}
//jcheng2020 revised end


void calibrateJoints(int* calJoints)
{
  //jcheng2020 revised
  bool repeat_flag[NUM_JOINTS] = {false, false ,false ,false ,false ,false};
  //jcheng2020_revised end
  
  // check which joints to calibrate
  bool calAllDone = false;
  bool calJointsDone[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    calJointsDone[i] = !calJoints[i];
  }
  
  // first pass of calibration, fast speed
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    stepperJoints[i].setSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i]);
  }
  while (!calAllDone)
  {
    calAllDone = true;
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
      // if joint is not calibrated yet
      if (!calJointsDone[i])
      {
        // check limit switches
        if (!reachedLimitSwitch(i))
        {
          // limit switch not reached, continue moving
          stepperJoints[i].runSpeed();
          calAllDone = false;
        }
        else
        {
          if(repeat_flag[i] == false)
          {
            calRetrieveTemp(i);
            repeat_flag[i] = true;
            
            if( i == 0 || i == 1|| i == 3 || i ==4)
            {
              stepperJoints[i].move(-ENC_RANGE_STEPS[i]/ ENC_MULT[i]);
            }
            else if (i == 2 || i == 5)
            {
              stepperJoints[i].move(ENC_RANGE_STEPS[i]/ ENC_MULT[i]);
            }
           
            stepperJoints[i].setSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i]);
            calAllDone = false;
          }
          else
          {
            // limit switch reached
            stepperJoints[i].setSpeed(0); // redundancy
            calJointsDone[i] = true;
          }
        }   
      }   
    } 
  }
  delay(2000);

  return;
}

bool reachedLimitSwitch(int joint)
{
  int pin = CAL_PINS[joint];
  // check multiple times to deal with noise
  // possibly EMI from motor cables?
  if (digitalRead(pin) == LIMIT_SWITCH_HIGH[joint])
  {
    if (digitalRead(pin) == LIMIT_SWITCH_HIGH[joint])
    {
      if (digitalRead(pin) == LIMIT_SWITCH_HIGH[joint])
      {
        if (digitalRead(pin) == LIMIT_SWITCH_HIGH[joint])
        {
          if (digitalRead(pin) == LIMIT_SWITCH_HIGH[joint])
          {
            return true;
          }
        }
      }
    }
  }
  return false;
}




//jcheng2020 revised begin
//retrive the joints for calibration process
void calRetrieve(int i, int curEncSteps[])
{
  bool restPosReached;
    stepperJoints[i].setAcceleration(MOTOR_MAX_ACCEL[i]);
    stepperJoints[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
    stepperJoints[i].move((REST_ENC_POSITIONS[i] - curEncSteps[i]) / ENC_MULT[i]);
    restPosReached = false;
    while (!restPosReached)
    {
      restPosReached = true;
       // read current joint positions
      readEncPos(curEncSteps);
      if (abs(REST_ENC_POSITIONS[i] - curEncSteps[i]) > 5)
      {
        restPosReached = false;
        stepperJoints[i].move((REST_ENC_POSITIONS[i] - curEncSteps[i]) / ENC_MULT[i]);
        stepperJoints[i].run();
      }
    }
}
//jcheng2020 revised end

void stateTRAJ()
{
  // clear message
  inData = "";

  // initialise joint steps
  int curEncSteps[NUM_JOINTS];
  readEncPos(curEncSteps);

  int cmdEncSteps[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    cmdEncSteps[i] = curEncSteps[i];
  }

  // initialise AccelStepper instance
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    stepperJoints[i] = AccelStepper(1, STEP_PINS[i], DIR_PINS[i]);
    //stepperJoints[i].setPinsInverted(true, false, false); // DM542T CW
    stepperJoints[i].setPinsInverted(false, false, false); // DM542T CW jcheng2020 revised
    stepperJoints[i].setAcceleration(MOTOR_MAX_ACCEL[i]);
    stepperJoints[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
  }
  //stepperJoints[3].setPinsInverted(false, false, false); // J4 DM320T CCW
  stepperJoints[3].setPinsInverted(true, false, false); // J4 DM320T CCW jcheng2020 revised

  // start loop
  while (STATE == STATE_TRAJ)
  {
    /*
    char received = "";
    // check for message from host
    if (Serial.available())
    {
      received = Serial.read();
      inData += received;
    }
    */

    // process message when new line character is received
    if (whetherReceive == true)
    {
      String function = inData.substring(0, 2);
      // update trajectory information
      if (function == "MT")
      {
        // read current joint positions
        readEncPos(curEncSteps);

        // update host with joint positions
        String msg = String("JP") + String("A") + String(curEncSteps[0]) + String("B") + String(curEncSteps[1]) + String("C") + String(curEncSteps[2])
                   + String("D") + String(curEncSteps[3]) + String("E") + String(curEncSteps[4]) + String("F") + String(curEncSteps[5]) + String("\n");
        //Serial.print(msg);
        str_msg.data = msg.c_str();
        chatter.publish( &str_msg );
        nh.spinOnce();

        // get new position commands
        int msgIdxJ1 = inData.indexOf('A');
        int msgIdxJ2 = inData.indexOf('B');
        int msgIdxJ3 = inData.indexOf('C');
        int msgIdxJ4 = inData.indexOf('D');
        int msgIdxJ5 = inData.indexOf('E');
        int msgIdxJ6 = inData.indexOf('F');
        cmdEncSteps[0] = inData.substring(msgIdxJ1 + 1, msgIdxJ2).toInt();
        cmdEncSteps[1] = inData.substring(msgIdxJ2 + 1, msgIdxJ3).toInt();
        cmdEncSteps[2] = inData.substring(msgIdxJ3 + 1, msgIdxJ4).toInt();
        cmdEncSteps[3] = inData.substring(msgIdxJ4 + 1, msgIdxJ5).toInt();
        cmdEncSteps[4] = inData.substring(msgIdxJ5 + 1, msgIdxJ6).toInt();
        cmdEncSteps[5] = inData.substring(msgIdxJ6 + 1).toInt();

        // update target joint positions jcheng2020 revised required
        //readEncPos(curEncSteps);
        //jcheng2020 revised
        //bool trigger = true;
        
        //while(trigger == true)
        //{
        //jcheng2020 revised end
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
          //jcheng2020 removed 
          //curEncSteps[1] = J2encPos.read() * ENC_DIR[1];
          //jcheng2020 removed end
          //jcheng2020 revised
          //trigger = false;
          //readEncPos(curEncSteps);
          //jcheng2020 revised end
          int diffEncSteps = cmdEncSteps[i] - curEncSteps[i];
          if (abs(diffEncSteps) > 2)
          {
            int diffMotSteps = diffEncSteps / ENC_MULT[i];
            if (diffMotSteps < MOTOR_STEPS_PER_REV[i])
            {
              // for the last rev of motor, introduce artificial decceleration
              // to help prevent overshoot
              diffMotSteps = diffMotSteps / 2;
            }
            stepperJoints[i].move(diffMotSteps);
            stepperJoints[i].run();
            //jcheng2020 revised
            //trigger = true;
            //jcheng2020 revised end
          }
        }
        //jcheng2020 revised
        //}
        //jcheng2020 revised end
      }
      else if (function == "JC")
      { //jcheng2020 revised begin
        bool restPosReached = false;
        // calibrate joint 1
        int calJoint1[] = { 1, 0, 0, 0, 0, 0 }; // 100000
        calibrateJoints(calJoint1);
        // record encoder steps
        int calStepJ1 = J1encPos.read() * ENC_DIR[0];

        //jcheng2020 revised
        J1encPos.write(0);
        //jcheng2020 disable encoders revise
        stepperJoints[0].setCurrentPosition(0/ENC_MULT[0]);
        //jcheng2020 disable encoders revise end
        readEncPos(curEncSteps);
        cmdEncSteps[0] = curEncSteps[0];
        calRetrieve(0, curEncSteps);
        
        
        // calibrate joint 2
        int calJoint2[] = { 0, 1, 0, 0, 0, 0 }; // 010000
        calibrateJoints(calJoint2);
        // record encoder steps
        int calStepJ2 = J2encPos.read() * ENC_DIR[1];
        
        //jcheng2020 revised
        J2encPos.write(0);
        //jcheng2020 disable encoders revise
        stepperJoints[1].setCurrentPosition(0/ENC_MULT[1]);
        //jcheng2020 disable encoders revise end
        readEncPos(curEncSteps);
        cmdEncSteps[1] = curEncSteps[1];
        calRetrieve(1, curEncSteps);
        
        // calibrate joint 3
        int calJoint3[] = { 0, 0, 1, 0, 0, 0 }; // 001000
        calibrateJoints(calJoint3);
        // record encoder steps
        int calStepJ3 = J3encPos.read() * ENC_DIR[2];
        
        //jcheng2020 revised
        //J3encPos.write(ENC_RANGE_STEPS[2]);
        J3encPos.write(ENC_RANGE_STEPS[2]);
        //jcheng2020 disable encoders revise
        stepperJoints[2].setCurrentPosition(ENC_RANGE_STEPS[2]/ENC_MULT[2]);
        //jcheng2020 disable encoders revise end
        readEncPos(curEncSteps);
        cmdEncSteps[2] = curEncSteps[2];
        calRetrieve(2, curEncSteps);
        
        // calibrate joint 4
        int calJoint4[] = { 0, 0, 0, 1, 0, 0 }; // 000100
        calibrateJoints(calJoint4);
        // record encoder steps
        int calStepJ4 = J4encPos.read() * ENC_DIR[3];
        
        //jcheng2020 revised
        J4encPos.write(0);
        //jcheng2020 disable encoders revise
        stepperJoints[3].setCurrentPosition(0/ENC_MULT[3]);
        //jcheng2020 disable encoders revise end
        readEncPos(curEncSteps);
        cmdEncSteps[3] = curEncSteps[3];
        calRetrieve(3, curEncSteps);
        
        // calibrate joint 5
        int calJoint5[] = { 0, 0, 0, 0, 1, 0 }; // 000010
        calibrateJoints(calJoint5);
        // record encoder steps
        int calStepJ5 = J5encPos.read() * ENC_DIR[4];

        //jcheng2020 revised
        J5encPos.write(0);
        //jcheng2020 disable encoders revise
        stepperJoints[4].setCurrentPosition(0/ENC_MULT[4]);
        //jcheng2020 disable encoders revise end
        readEncPos(curEncSteps);
        cmdEncSteps[4] = curEncSteps[4];
        calRetrieve(4, curEncSteps);
        
        // calibrate joint 6
        int calJoint6[] = { 0, 0, 0, 0, 0, 1 }; // 000001
        calibrateJoints(calJoint6);
        // record encoder steps
        int calStepJ6 = J6encPos.read() * ENC_DIR[5];

        //jcheng2020 revised
        J6encPos.write(ENC_RANGE_STEPS[5]);
        //jcheng2020 disable encoders revise
        stepperJoints[5].setCurrentPosition(ENC_RANGE_STEPS[5]/ENC_MULT[5]);
        //jcheng2020 disable encoders revise end
        readEncPos(curEncSteps);
        cmdEncSteps[5] = curEncSteps[5];
        calRetrieve(5, curEncSteps);

        //jcheng2020 revised end
        /*
        // calibrate joint 6
        int calJoint6[] = { 0, 0, 0, 0, 0, 1 }; // 000001
        calibrateJoints(calJoint6);

        // record encoder steps
        int calStepJ6 = J6encPos.read() * ENC_DIR[5];

        // calibrate joints 1 to 5
        int calJoints[] = { 1, 1, 1, 1, 1, 0 }; // 111110
        calibrateJoints(calJoints);

        // record encoder steps
        int calStepJ1 = J1encPos.read() * ENC_DIR[0];
        int calStepJ2 = J2encPos.read() * ENC_DIR[1];
        int calStepJ3 = J3encPos.read() * ENC_DIR[2];
        int calStepJ4 = J4encPos.read() * ENC_DIR[3];
        int calStepJ5 = J5encPos.read() * ENC_DIR[4];
        */
        /*
        // if limit switch at lower end, set encoder to 0
        // otherwise set to encoder upper limit
        J1encPos.write(0);
        J2encPos.write(0);
        J3encPos.write(ENC_RANGE_STEPS[2]);
        J4encPos.write(0);
        J5encPos.write(0);
        J6encPos.write(ENC_RANGE_STEPS[5]);

        // read current joint positions
        readEncPos(curEncSteps);

        // return to original position
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            stepperJoints[i].setAcceleration(MOTOR_MAX_ACCEL[i]);
            stepperJoints[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
            cmdEncSteps[i] = curEncSteps[i];
            stepperJoints[i].move((REST_ENC_POSITIONS[i] - curEncSteps[i]) / ENC_MULT[i]);
        }

        bool restPosReached = false;
        while (!restPosReached)
        {
          restPosReached = true;
          // read current joint positions
          readEncPos(curEncSteps);
          for (int i = 0; i < NUM_JOINTS; ++i)
          {
            if (abs(REST_ENC_POSITIONS[i] - curEncSteps[i]) > 5)
            {
              restPosReached = false;
              stepperJoints[i].move((REST_ENC_POSITIONS[i] - curEncSteps[i]) / ENC_MULT[i]);
              stepperJoints[i].run();
            }
          }
        }
        */
        
        // calibration done, send calibration values
        String msg = String("JC") + String("A") + String(calStepJ1) + String("B") + String(calStepJ2) + String("C") + String(calStepJ3)
                  + String("D") + String(calStepJ4) + String("E") + String(calStepJ5) + String("F") + String(calStepJ6) + String("\n");
        //Serial.print(msg);
        str_msg.data = msg.c_str();
        chatter.publish( &str_msg );
        nh.spinOnce();

        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            stepperJoints[i].setAcceleration(MOTOR_MAX_ACCEL[i]);
            stepperJoints[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
        }
      }
      else if (function == "JP")
      {
        // read current joint positions
        readEncPos(curEncSteps);

        // update host with joint positions
        String msg = String("JP") + String("A") + String(curEncSteps[0]) + String("B") + String(curEncSteps[1]) + String("C") + String(curEncSteps[2])
                   + String("D") + String(curEncSteps[3]) + String("E") + String(curEncSteps[4]) + String("F") + String(curEncSteps[5]) + String("\n");
        //Serial.print(msg);
        str_msg.data = msg.c_str();
        chatter.publish( &str_msg );
        nh.spinOnce();
      }
      else if (function == "SS")
      {
        updateStepperSpeed(inData);
        // set motor speed and acceleration
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
          MOTOR_MAX_SPEED[i] = JOINT_MAX_SPEED[i] * ENC_STEPS_PER_DEG[i] / ENC_MULT[i];
          MOTOR_MAX_ACCEL[i] = JOINT_MAX_ACCEL[i] * ENC_STEPS_PER_DEG[i] / ENC_MULT[i];
          stepperJoints[i].setAcceleration(MOTOR_MAX_ACCEL[i] * MOTOR_ACCEL_MULT[i]);
          stepperJoints[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
        }
        // read current joint positions
        readEncPos(curEncSteps);

        // update host with joint positions
        String msg = String("JP") + String("A") + String(curEncSteps[0]) + String("B") + String(curEncSteps[1]) + String("C") + String(curEncSteps[2])
                   + String("D") + String(curEncSteps[3]) + String("E") + String(curEncSteps[4]) + String("F") + String(curEncSteps[5]) + String("\n");
        //Serial.print(msg);
        str_msg.data = msg.c_str();
        chatter.publish( &str_msg );
        nh.spinOnce();
      }
      else if (function == "ST")
      {
        if (!initStateTraj(inData))
        {
          //STATE = STATE_ARCS;
          return;
        }

      }
      
      // clear message
      inData = "";
      whetherReceive = false;
    }
    // execute motor commands
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
      // target joint positions are already updated, just call run()
      stepperJoints[i].run();
    }
    nh.spinOnce();
    
  }
}




void stateERR()
{
  // enter holding state
  digitalWrite(J1stepPin, LOW);
  digitalWrite(J2stepPin, LOW);
  digitalWrite(J3stepPin, LOW);
  digitalWrite(J4stepPin, LOW);
  digitalWrite(J5stepPin, LOW);
  digitalWrite(J6stepPin, LOW);

  // do recovery
  while (STATE == STATE_ERR) {}
}

void loop() 
{  
  nh.spinOnce();
  //test traj state
  // STATE = STATE_TRAJ;

  // state control
  switch (STATE)
  {
    case STATE_ARCS:
      break;
    case STATE_TRAJ:
      stateTRAJ();
      break;
    case STATE_ERR:
      stateERR();
      break;
    default:
      stateTRAJ();
      break;
  }
}
