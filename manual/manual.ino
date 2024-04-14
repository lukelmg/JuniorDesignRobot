#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <AccelStepper.h>
#include <Stepper.h>
#include "Adafruit_TCS34725.h"
#include <SpeedyStepper.h>

#define motorPin1_RA1 11
#define motorPin2_RA1 12

#define motorPin1_RA2 8
#define motorPin2_RA2 9

#define motorPin1_P1 6
#define motorPin2_P1 7


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
byte gammatable[256];
#define commonAnode true

char detectColor() {
  float red, green, blue;

  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);

  tcs.setInterrupt(true);  // turn off LED

  Serial.print("R:\t");
  Serial.print(int(red));
  Serial.print("\tG:\t");
  Serial.print(int(green));
  Serial.print("\tB:\t");
  Serial.print(int(blue));

  if (red > green && red > blue) {
    return "R";
  } else if (green > red && green > blue) {
    return "G";
  } else if (blue > red && blue > green) {
    return "B";
  } else {
    return "U";
  }
}

int Rcount = 0;
int Gcount = 0;
int Bcount = 0;

AccelStepper accelP1 = AccelStepper(AccelStepper::DRIVER, motorPin1_P1, motorPin2_P1);

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

  moveXYWithAbsoluteCoordination(theta1Deg, theta2Deg, myspeed, myspeed / 1.3);
}

void moveP1(float Y) {
  //float y = Y * 200 / 5;
  //stepper_P1.moveToPositionInSteps(y);
  accelP1.moveTo(-Y * 200 / 5);
  accelP1.runToPosition();
}

const int joyLXpin = 13;
const int joyLYpin = 10;
const int joyRXpin = 8;
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
const int gripperOpenPos = 60;
const int gripperClosedPos = 180;

const int GripperPin = 5;

int speedAccel = 7000;

void GripperOpen() {
  Gripper.write(gripperOpenPos);
}

void GripperClose() {
  Gripper.write(gripperClosedPos);
}

  int P1max = 1500;

void setup() {
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

  const float maxHomingDistanceInMM = 20000;

  stepper_RA1.moveToHomeInSteps(1, 800, maxHomingDistanceInMM, 23);
  stepper_RA2.moveToHomeInSteps(-1, 800, maxHomingDistanceInMM, 33);
  stepper_P1.moveToHomeInSteps(-1, 600, 10000, 27);


  accelP1.setMaxSpeed(P1max);      //  steps/s
  accelP1.setAcceleration(750);  // steps/s^2

  moveP1(20);

  pinMode(motorPin1_P1, OUTPUT);
  pinMode(motorPin2_P1, OUTPUT);

  delay(500);

  //Serial.begin(9600);
}

float curX = 100.0;
float curZ = 100.0;

void loop() {
  joyLX = analogRead(joyLXpin);
  joyLY = analogRead(joyLYpin);
  joyRX = analogRead(joyRXpin);
  joyRY = analogRead(joyRYpin);

  buttonL = digitalRead(buttonLpin);
  buttonR = digitalRead(buttonRpin);

  //String output = "LX: " + String(joyLX) + ", LY: " + String(joyLY) + ", RX: " + String(joyRX) + ", RY: " + String(joyRY) + ", ButtonL: " + String(buttonL) + ", ButtonR: " + String(buttonR);
  //Serial.println(output);

  int P1speed = map(joyRX, 0, 1023, -P1max, P1max);
  float Xspeed = map(joyLY, 0, 1023, -100, 100);
  float Zspeed = map(joyLX, 0, 1023, 100, -100);

  if (abs(P1speed) < 500) {
    P1speed = 0;
  }

  float eqSpeed = pow((abs(P1speed) / 50),2.15);

  if (P1speed < 0) {
    eqSpeed *= -1;
  }

  accelP1.setSpeed(eqSpeed);
  accelP1.runSpeed();

  //output = "P1 Speed: " + String(P1speed) + ", X Speed: " + String(Xspeed) + ", Yspeed: " + String(Yspeed);
  //Serial.println(output);


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

  moveToXY(curX, curZ, 7000);
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
    float scaler = (float)absSteps2 / (float)absSteps1;
    speedInStepsPerSecond_2 = speedInStepsPerSecond_2 * scaler;
    accelerationInStepsPerSecondPerSecond_2 = accelerationInStepsPerSecondPerSecond_2 * scaler;
  }

  if ((absSteps2 > absSteps1) && (moveSteps2 != 0)) {
    float scaler = (float)absSteps1 / (float)absSteps2;
    speedInStepsPerSecond_1 = speedInStepsPerSecond_1 * scaler;
    accelerationInStepsPerSecondPerSecond_1 = accelerationInStepsPerSecondPerSecond_1 * scaler;
  }

  stepper_RA1.setSpeedInStepsPerSecond(speedInStepsPerSecond_1);
  stepper_RA1.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_1);
  stepper_RA1.setupRelativeMoveInSteps(moveSteps1);

  stepper_RA2.setSpeedInStepsPerSecond(speedInStepsPerSecond_2);
  stepper_RA2.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_2);
  stepper_RA2.setupRelativeMoveInSteps(moveSteps2);

  while ((!stepper_RA1.motionComplete()) || (!stepper_RA2.motionComplete())) {
    stepper_RA1.processMovement();
    stepper_RA2.processMovement();
  }

  currentAngle1 += moveAngle1;
  currentAngle2 += moveAngle2;
  count++;
}
