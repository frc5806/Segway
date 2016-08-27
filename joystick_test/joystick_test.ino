#include "joystick.h"

void setup() {
  Serial.begin(9600);
}

void loop() {
  Position pos = getPosition();
  MotorValues mvs = getMotorValues();
  /*Serial.print(pos.x);
  Serial.print(", ");
  Serial.print(pos.y);
  Serial.print(", ");*/
  Serial.print(pos.x);
  Serial.print(", ");
  Serial.print(pos.y);
  Serial.print(", ");
  Serial.print(mvs.left);
  Serial.print(", ");
  Serial.print(mvs.right);
  Serial.print("\n");

  // for testing the motors themselves
  analogWrite(9, mvs.left);
  analogWrite(10, mvs.right);
  delay(50);
}

