#include <Servo.h>

/************************
*********JOYSTICK********
*************************/
#define JOYSTICK_X_PIN A0 
#define JOYSTICK_Y_PIN A1

int* getJoystickCoords() {
  int joystickCoords[2];
  joystickCoords[0] = analogRead(JOYSTICK_X_PIN);
  joystickCoords[1] = analogRead(JOYSTICK_Y_PIN);

  return joystickCoords;
}

/************************
*********MOTORS**********
*************************/
#define RIGHT_MOTOR_PIN 9  
#define LEFT_MOTOR_PIN 11

Servo rightMotor;
Servo leftMotor;

void attachMotors() {
  rightMotor.attach(RIGHT_MOTOR_PIN);
  leftMotor.attach(LEFT_MOTOR_PIN);
}

void setMotors(float r, float l) {
  rightMotor.write(-r*90+90); 
  leftMotor.write(l*90+90); 
}

/************************
*********IMU*************
*************************/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include <Wire.h>
#endif

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void imuSetup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

float getPitch() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return -999;

    do {
      while (!mpuInterrupt && fifoCount < packetSize) {};
      mpuIntStatus = mpu.getIntStatus();
    } while (mpuIntStatus & 0x02);

    mpuInterrupt = false;

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    return ypr[1] * 180/M_PI;
}

/**********************
*****SAFETY BUTTON*****
***********************/

#define SAFETY_BUTTON_PIN 7

void setupButton() {
  pinMode(SAFETY_BUTTON_PIN, INPUT);
}

bool isSafe() {
  return digitalRead(SAFETY_BUTTON_PIN) == HIGH;
}

/************************
*********PID*************
*************************/

#define MAX_OFFSET 20.0
#define PROPORTIONAL_TERM 1.0

int CENTER;

void calibrateIMU() {
  for(int a = 0; a < 1200; a++) {
    getPitch();
    delay(10);
  }
  CENTER = getPitch();
}

int getMotorValue() {
  float pitch = getPitch();
  
  Serial.print(pitch);
  Serial.print(" ");
  
  float offset = CENTER - pitch;

  Serial.print(offset);
  Serial.print(" ");

  return PROPORTIONAL_TERM*offset/(float)MAX_OFFSET; 
}

/************************
*********MAIN************
*************************/

void setup()  {
  // Setup serial
  Serial.begin(115200);
  while (!Serial);

  setupButton();
  attachMotors();
  imuSetup();
  calibrateIMU();
} 

void loop()  {
  int motorValue = getMotorValue();
  //int* coords = getJoystickCoords();

  /*Serial.print(coords[0]);
  Serial.print(" ");
  Serial.print(coords[1]);
  Serial.print(", ");
  */
  //Serial.print(coords[0]);
  //Serial.print(", ");
  Serial.print(motorValue);
  Serial.print("\n");

  setMotors(motorValue, motorValue);
  delay(5);
 }

