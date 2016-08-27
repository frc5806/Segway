#define MOTOR_1_PIN 10
#define MOTOR_2_PIN 11

void setup()  { 

} 

void loop()  { 
  int leftMotor = 255;
  int rightMotor = 0;


  analogWrite(MOTOR_1_PIN, leftMotor);
  analogWrite(MOTOR_2_PIN, rightMotor);
}

