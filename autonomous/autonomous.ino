#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <AccelStepper.h>
#include <Stepper.h>
#include "Adafruit_TCS34725.h"
#include <SpeedyStepper.h>

#define motorPin1_RA1 12
#define motorPin2_RA1 11

#define motorPin1_RA2 10
#define motorPin2_RA2 9

#define motorPin1_P1 7
#define motorPin2_P1 6

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
byte gammatable[256];
#define commonAnode true

char detectColor() {
  float red, green, blue;

  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);

  tcs.setInterrupt(true);  // turn off LED

  if (red > green && red > blue) {
    return 'R';  // Note the single quotes, which denote a char
  } else if (green > red && green > blue) {
    return 'G';  // Note the single quotes, which denote a char
  } else if (blue > red && blue > green) {
    return 'B';  // Note the single quotes, which denote a char
  } else {
    return 'U';  // Note the single quotes, which denote a char for 'Unknown'
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

  Serial.println(theta1Deg);

  moveXYWithAbsoluteCoordination(theta1Deg, theta2Deg, myspeed, myspeed / 1.3);
}

void moveP1(float Y) {
  float y = Y * 200 / 5 / 3;
  stepper_P1.moveToPositionInSteps(y);
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

void setup() {
  Serial.begin(9600);
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1)
      ;
  }

  for (int i = 0; i < 256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
  }

  delay(3000);

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

  stepper_P1.setSpeedInStepsPerSecond(700);
  stepper_P1.setAccelerationInStepsPerSecondPerSecond(750);

  const float maxHomingDistanceInMM = 20000;

  GripperOpen();
  GripperClose();
  GripperOpen();
  GripperClose();

  stepper_RA1.moveToHomeInSteps(1, 800, maxHomingDistanceInMM, 23);
  stepper_RA2.moveToHomeInSteps(-1, 800, maxHomingDistanceInMM, 33);

  moveToXY(100.0, 50.0, 1000);

  stepper_P1.moveToHomeInSteps(-1, 400, 10000, 27);

  moveP1(32.0);

  pinMode(motorPin1_P1, OUTPUT);
  pinMode(motorPin2_P1, OUTPUT);

  delay(500);

  GripperOpen();

  delay(10000);


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

  const float redCoordinates[3][3] = {
    { 212.0, 262.3, 200.0 }, { 212.0, 262.3, 110.0 }, { 212.0, 262.3, 20.0 }
  };

  const float greenCoordinates[3][3] = {
    { 212.0, 133.52, 200.0 }, { 212.0, 133.52, 110.0 }, { 212.0, 133.52, 110.0 }
  };

  const float blueCoordinates[3][3] = {
    { 212.0, 390.78, 200.0 }, { 212.0, 390.78, 110.0 }, { 212.0, 390.78, 20.0 }
  };

  int goSpeed = 7000;

  float p1offset = 160.0;

  float Xsafe = 50.0;

  int gripperOpenDelay = 1000;

  int globalDelay = 50;

  for (int i = 0; i < 9; i++) {
    GripperOpen();
    moveToXY(blockLocations[i][0] - 30.0, blockLocations[i][1], goSpeed);
    delay(300);
    GripperClose();
    delay(150);
    moveToXY(blockLocations[i][0] - 30.0, blockLocations[i][1] + 30.0, goSpeed);
    moveToXY(100.0, 50.0, goSpeed);
    // go to grid
    char currentColor = detectColor();

    Serial.println(currentColor);

    if (currentColor == 'R') {
      moveP1(redCoordinates[Rcount][1] + p1offset);
      delay(globalDelay);
      moveToXY(redCoordinates[Rcount][0] - Xsafe, redCoordinates[Rcount][2], goSpeed);
      delay(globalDelay);
      moveToXY(redCoordinates[Rcount][0], redCoordinates[Rcount][2], goSpeed);
      GripperOpen();
      delay(gripperOpenDelay);
      moveToXY(redCoordinates[Rcount][0] - Xsafe, redCoordinates[Rcount][2], goSpeed);
      Rcount++;
    } else if (currentColor == 'G') {
      moveP1(greenCoordinates[Gcount][1] + p1offset);
      delay(globalDelay);
      moveToXY(greenCoordinates[Gcount][0] - Xsafe, greenCoordinates[Gcount][2], goSpeed);
      delay(globalDelay);
      moveToXY(greenCoordinates[Gcount][0], greenCoordinates[Gcount][2], goSpeed);
      GripperOpen();
      delay(gripperOpenDelay);
      moveToXY(greenCoordinates[Gcount][0] - Xsafe, greenCoordinates[Gcount][2], goSpeed);
      Gcount++;
    } else if (currentColor == 'B') {
      moveP1(blueCoordinates[Bcount][1] + p1offset);
      delay(globalDelay);
      moveToXY(blueCoordinates[Bcount][0] - Xsafe, blueCoordinates[Bcount][2], goSpeed);
      delay(globalDelay);
      moveToXY(blueCoordinates[Bcount][0], blueCoordinates[Bcount][2], goSpeed);
      GripperOpen();
      delay(gripperOpenDelay);
      moveToXY(blueCoordinates[Bcount][0] - Xsafe, blueCoordinates[Bcount][2], goSpeed);
      Bcount++;
    }

    delay(globalDelay);

    moveToXY(100.0, 50.0, goSpeed);

    moveP1(32.0);
  }
}

float curX = 95.174;
float curZ = 271.962;

void loop() {
  char currentColor = detectColor();
  Serial.println(currentColor);
  delay(100);
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
  float moveAngle1 = 0.0;
  float moveAngle2 = 0.0;
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

  Serial.println(moveAngle1);

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

  Serial.println(moveSteps1);

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
