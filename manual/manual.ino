#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <AccelStepper.h>
#include <Stepper.h>

#include <SpeedyStepper.h>

#define motorPin1_RA1 12
#define motorPin2_RA1 11

#define motorPin1_RA2 10
#define motorPin2_RA2 9

#define motorPin1_P1 7
#define motorPin2_P1 6

AccelStepper accelP1 = AccelStepper(AccelStepper::DRIVER, motorPin1_P1, motorPin2_P1);
//Stepper stepperHP1(200, motorPin1_P1, motorPin2_P1);

SpeedyStepper stepper_RA1;
SpeedyStepper stepper_RA2;
SpeedyStepper stepper_P1;

const float L1 = 200.0;
const float L2 = 150.0;

void moveToXY(float X, float Z, int myspeed) {
  const float radToDeg = 180.0 / PI;

  float x = static_cast<float>(X);
  float z = static_cast<float>(Z);

  const float pi = 3.14159265;

  float theta2 = acos((x * x + z * z - L1 * L1 - L2 * L2) / (2 * L1 * L2));
  float theta1 = atan2(z, x) - atan2((L2 * sin(2 * pi - theta2)), (L1 + L2 * cos(2 * pi - theta2)));

  float theta1Deg = theta1 * radToDeg;
  float theta2Deg = theta2 * radToDeg;

  moveXYWithAbsoluteCoordination(theta1Deg, theta2Deg, myspeed, myspeed);
}


void moveP1(float Y) {
  float y = Y * 200 / 5;
  stepper_P1.moveToPositionInSteps(y);
}

void manualP1(int delayTime) {
}

const int joyLXpin = 12;
const int joyLYpin = 13;
const int joyRXpin = 14;
const int joyRYpin = 15;

int joyLX = 0;
int joyLY = 0;
int joyRX = 0;
int joyRY = 0;

const int buttonLpin = 41;
const int buttonRpin = 43;

int buttonL = 0;
int buttonR = 0;

Servo Gripper;
const int gripperOpenPos = 49;
const int gripperClosedPos = 180;

const int GripperPin = 5;

int speedAccel = 7000;

void GripperOpen() {
  Gripper.write(gripperOpenPos);
}

void GripperClose() {
  Gripper.write(gripperClosedPos);
}

void setup() {
  //Serial.begin(9600);
  Gripper.attach(GripperPin);

  pinMode(buttonLpin, INPUT_PULLUP);
  pinMode(buttonRpin, INPUT_PULLUP);

  stepper_RA1.connectToPins(motorPin1_RA1, motorPin2_RA1);
  stepper_RA2.connectToPins(motorPin1_RA2, motorPin2_RA2);
  stepper_P1.connectToPins(motorPin1_P1, motorPin2_P1);

  pinMode(23, INPUT_PULLUP);
  pinMode(27, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);

  stepper_RA1.setSpeedInStepsPerSecond(speedAccel);
  stepper_RA1.setAccelerationInStepsPerSecondPerSecond(speedAccel);
  stepper_RA2.setSpeedInStepsPerSecond(speedAccel);
  stepper_RA2.setAccelerationInStepsPerSecondPerSecond(speedAccel);

  stepper_P1.setSpeedInStepsPerSecond(1000);
  stepper_P1.setAccelerationInStepsPerSecondPerSecond(5000);

  const float homingSpeedInMMPerSec = 125;
  const float maxHomingDistanceInMM = 20000;

  stepper_RA1.moveToHomeInSteps(1, homingSpeedInMMPerSec, maxHomingDistanceInMM, 23);
  stepper_RA2.moveToHomeInSteps(-1, homingSpeedInMMPerSec, maxHomingDistanceInMM, 33);
  stepper_P1.moveToHomeInSteps(1, 500, 10000, 27);

  pinMode(motorPin1_P1, OUTPUT);
  pinMode(motorPin2_P1, OUTPUT);

  float blockLocations[9][2] = {
    { 205.2, 66.2 },
    { 205.2, 40.8 },
    { 205.2, 15.4 },
    { 270.6, 66.2 },
    { 270.6, 40.8 },
    { 270.6, 15.4 },
    { 336, 66.2 },
    { 336, 40.8 },
    { 336, 15.4 }
  };

  int goSpeed = 7000;

  for (int i = 0; i < 9; i++) {
    if (i > 5) {
      goSpeed = 3000;
    }

    moveP1(-20.0);
    GripperOpen();
    moveToXY(blockLocations[i][0] - 30.0, blockLocations[i][1] + 5.0, goSpeed);
    GripperClose();
    delay(500);
    moveToXY(blockLocations[i][0] - 30.0, blockLocations[i][1] + 30.0, goSpeed);
    moveToXY(100.0, 260.0, goSpeed);
    GripperOpen();
    delay(500);
  }
}

float curX = 95.174;
float curZ = 271.962;

void loop() {
  joyLX = analogRead(joyLXpin);
  joyLY = analogRead(joyLYpin);
  joyRX = analogRead(joyRXpin);
  joyRY = analogRead(joyRYpin);

  buttonL = digitalRead(buttonLpin);
  buttonR = digitalRead(buttonRpin);

  //String output = "LX: " + String(joyLX) + ", LY: " + String(joyLY) + ", RX: " + String(joyRX) + ", RY: " + String(joyRY) + ", ButtonL: " + String(buttonL) + ", ButtonR: " + String(buttonR);
  //Serial.println(output);

  // map joysticks based on anlog input
  int P1speed = map(joyRX, 0, 1023, 2000, -2000);
  float Xspeed = map(joyLY, 0, 1023, -100, 100);
  float Zspeed = map(joyRY, 0, 1023, 100, -100);

  //output = "P1 Speed: " + String(P1speed) + ", X Speed: " + String(Xspeed) + ", Yspeed: " + String(Yspeed);
  //Serial.println(output);

  /*

  if (buttonL == 0) {
    GripperOpen();
  } else if (buttonR == 0) {
    GripperClose();
  }

  int moveMultiplier = 125;

  if (Xspeed > 30) {
    curX += Xspeed / moveMultiplier;
  } else if (Xspeed < -30) {
    curX += Xspeed / moveMultiplier;
  }

  if (Zspeed > 30) {
    curZ += Zspeed / moveMultiplier;
  } else if (Zspeed < -30) {
    curZ += Zspeed / moveMultiplier;
  }

  //moveToXY(curX, curZ);

  */

  if (P1speed < 0) {
    digitalWrite(motorPin2_P1, LOW);
  } else {
    digitalWrite(motorPin2_P1, HIGH);
  }

  int pulseTime = 800;

  if (abs(P1speed) > 500) {
    digitalWrite(motorPin1_P1, HIGH);
    delayMicroseconds(pulseTime);
    digitalWrite(motorPin1_P1, LOW);
    delayMicroseconds(pulseTime);
  }
}


float currentAngle1 = 100.0;
float currentAngle2 = 70.0;

int count = 0;


void moveXYWithAbsoluteCoordination(float targetPosition1, float targetPosition2, float speedInStepsPerSecond, float accelerationInStepsPerSecondPerSecond) {
  const float stepsPerRevolution = 200.0 * 4.0 * 4.0;  // 200 steps per rev, 4x gear ratio, 4x microstepping
  float speedInStepsPerSecond_1;
  float accelerationInStepsPerSecondPerSecond_1;
  float speedInStepsPerSecond_2;
  float accelerationInStepsPerSecondPerSecond_2;
  float moveAngle1 = 0;
  float moveAngle2 = 0;
  int absSteps1;
  int absSteps2;
  float ra1Offset;

  speedInStepsPerSecond_1 = speedInStepsPerSecond;
  accelerationInStepsPerSecondPerSecond_1 = accelerationInStepsPerSecondPerSecond;

  speedInStepsPerSecond_2 = speedInStepsPerSecond;
  accelerationInStepsPerSecondPerSecond_2 = accelerationInStepsPerSecondPerSecond;

  moveAngle1 = targetPosition1 - currentAngle1;
  moveAngle2 = targetPosition2 - currentAngle2;

  float realMoveAngle2 = moveAngle2 - moveAngle1;

  //Serial.println(realMoveAngle2);
  //Serial.println(moveAngle1);
  //Serial.println(moveAngle2);


  int moveSteps1 = static_cast<int>(round((moveAngle1 * stepsPerRevolution / 360.0)));
  int moveSteps2 = static_cast<int>(round((realMoveAngle2 * stepsPerRevolution / 360.0)));

  if (moveSteps1 >= 0)
    absSteps1 = moveSteps1;
  else
    absSteps1 = -moveSteps1;

  if (moveSteps2 >= 0)
    absSteps2 = moveSteps2;
  else
    absSteps2 = -moveSteps2;

  if ((absSteps1 > absSteps2) && (moveSteps1 != 0)) {
    //
    // slow down the motor traveling less far
    //
    float scaler = (float)absSteps2 / (float)absSteps1;
    speedInStepsPerSecond_2 = speedInStepsPerSecond_2 * scaler;
    accelerationInStepsPerSecondPerSecond_2 = accelerationInStepsPerSecondPerSecond_2 * scaler;
  }

  if ((absSteps2 > absSteps1) && (moveSteps2 != 0)) {
    //
    // slow down the motor traveling less far
    //
    float scaler = (float)absSteps1 / (float)absSteps2;
    speedInStepsPerSecond_1 = speedInStepsPerSecond_1 * scaler;
    accelerationInStepsPerSecondPerSecond_1 = accelerationInStepsPerSecondPerSecond_1 * scaler;
  }

  stepper_RA1.setSpeedInStepsPerSecond(speedInStepsPerSecond_1);
  stepper_RA1.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_1);
  stepper_RA1.setupRelativeMoveInSteps(moveSteps1);  // Hypothetical function for absolute moves

  stepper_RA2.setSpeedInStepsPerSecond(speedInStepsPerSecond_2);
  stepper_RA2.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_2);
  stepper_RA2.setupRelativeMoveInSteps(moveSteps2);  // Hypothetical function for absolute moves

  //
  // now execute the moves, looping until both motors have finished
  //
  while ((!stepper_RA1.motionComplete()) || (!stepper_RA2.motionComplete())) {
    stepper_RA1.processMovement();
    stepper_RA2.processMovement();
  }

  currentAngle1 += moveAngle1;
  currentAngle2 += moveAngle2;
  count++;
}
