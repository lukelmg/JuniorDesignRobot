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
  accelP1.moveTo(-Y * 200 / 5 / 3);
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

int P1max = 3000;

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

  accelP1.setMaxSpeed(P1max);     //  steps/s
  accelP1.setAcceleration(4000);  // steps/s^2

  stepper_P1.setSpeedInStepsPerSecond(1200);
  stepper_P1.setAccelerationInStepsPerSecondPerSecond(4000);

  stepper_RA1.moveToHomeInSteps(1, 800, maxHomingDistanceInMM, 23);
  stepper_RA2.moveToHomeInSteps(-1, 800, maxHomingDistanceInMM, 33);
  stepper_P1.moveToHomeInSteps(-1, 1000, 10000, 27);

  moveP1(20);

  pinMode(motorPin1_P1, OUTPUT);
  pinMode(motorPin2_P1, OUTPUT);

  delay(500);

  //Serial.begin(9600);
  stepper_P1.setAccelerationInStepsPerSecondPerSecond(10000);

  GripperOpen();
}

float curX = 100.0;
float curZ = 100.0;

int P1maxSpeed = 60000;

int state = 1;
int prevstate = state;
int P1determinedSpeed = 300;
int P1steps = 500;
int P1speed = 0;

void loop() {
  joyRX = analogRead(joyRXpin);
  P1speed = map(joyRX, 0, 1023, -1500, 1500);
  buttonL = digitalRead(buttonLpin);

  if (buttonL == 0) {
    stepper_P1.setSpeedInStepsPerSecond(350);
  } else {
    stepper_P1.setSpeedInStepsPerSecond(1200);
  }

  if (abs(P1speed) > 100) {
    //stepper_P1.moveRelativeInSteps(P1speed);
    if (P1speed > 0) {
      stepper_P1.setupRelativeMoveInSteps(-1000000000);
    } else {
      stepper_P1.setupRelativeMoveInSteps(1000000000);
    }
    while (!stepper_P1.processMovement()) {
      joyRX = analogRead(joyRXpin);
      P1speed = map(joyRX, 0, 1023, -1500, 1500);
      //stepper_P1.setSpeedInStepsPerSecond(abs(P1speed));
      if (abs(P1speed) < 50) {
        break;
      }
    }
  } else {
    float Xspeed = map(joyLY, 0, 1023, 100, -100);
    float Zspeed = map(joyLX, 0, 1023, -100, 100);

    joyLY = analogRead(joyLXpin);
    joyLX = analogRead(joyLYpin);
    joyRY = analogRead(joyRYpin);

    buttonR = digitalRead(buttonRpin);
    if (buttonR == 0 && state == 0) {
      GripperOpen();
      state = 1;
      delay(500);
    } else if (buttonR == 0 && state == 1) {
      GripperClose();
      state = 0;
      delay(500);
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

  //output = "P1 Speed: " + String(P1speed) + ", X Speed: " + String(Xspeed) + ", Yspeed: " + String(Yspeed);
  //Serial.println(output);
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
