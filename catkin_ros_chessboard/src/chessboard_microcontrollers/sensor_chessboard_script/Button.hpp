//<description>An Arduino script for receiving sensor reading from the sensor chessboard to detect the positions of chess pieces and transmitting the data to ROS subscribers</description>
//<maintainer email="jcheng2020@my.fit.edu">Junfu Cheng</maintainer>
//<author email="jcheng2020@my.fit.edu">Junfu Cheng</author>
#ifndef Button_hpp
#define Button_hpp
#include "Arduino.h"

enum Instruction {Wait, Submit};
class Button
{
  public:
    Button(int buttonPin);
    Instruction update();
  private:
    long long time;
    long long debounceTime;
    bool state;
    int buttonPin;
};
#endif
