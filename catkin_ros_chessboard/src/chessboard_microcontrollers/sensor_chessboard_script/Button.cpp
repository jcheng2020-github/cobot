#include "Button.hpp"


Button::Button(int buttonPin):state(false)
{
  this->buttonPin = buttonPin;
  pinMode(buttonPin, INPUT);
  time = millis();
}

Instruction Button::update()
{
  Instruction result = Wait;
  if(millis() - time > 100)
  {
    bool reading = digitalRead(buttonPin);
    if(reading == true && state == false)
    {
      result = Submit;
    }
    state = reading;
    time = millis();
  }
  return result;
}
