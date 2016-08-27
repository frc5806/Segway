#include "joystick.h"
#define MOTOR_1_PIN 10
#define MOTOR_2_PIN 11

void setup()  { 

} 

void loop()  { 
//  Code for running off of the joystick input
  MotorValues values = getMotorValues();
  analogWrite(MOTOR_1_PIN, values.left);
  analogWrite(MOTOR_2_PIN, values.right);
}

