#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <AccelStepper.h>
#include <Stepper.h>
#include <SpeedyStepper.h>

#define motorPin1_RA1 11
#define motorPin2_RA1 12

#define motorPin1_RA2 8
#define motorPin2_RA2 9

#define motorPin1_P1 6
#define motorPin2_P1 7

#define buttonRpin 43

const int realP1Speed = 2000;
const int realP1Accel = 3000;

float fract(float x) {
  return x - int(x);
}

float mix(float a, float b, float t) {
  return a + (b - a) * t;
}

float step(float e, float x) {
  return x < e ? 0.0 : 1.0;
}

float rgb2hsv(float r, float g, float b) {
  float s = step(b, g);
  float px = mix(b, g, s);
  float py = mix(g, b, s);
  float pz = mix(-1.0, 0.0, s);
  float pw = mix(0.6666666, -0.3333333, s);
  s = step(px, r);
  float qx = mix(px, r, s);
  float qz = mix(pw, pz, s);
  float qw = mix(r, px, s);
  float d = qx - min(qw, py);
  float hsv = abs(qz + (qw - py) / (6.0 * d + 1e-10));
  //hsv[1] = d / (qx + 1e-10);
  //hsv[2] = qx;
  return hsv;
}

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
byte gammatable[256];
#define commonAnode true

char detectColor() {
  float red, green, blue;

  tcs.setInterrupt(true);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);

  tcs.setInterrupt(true);  // turn off LED

  float hue = rgb2hsv(red, green, blue);
  Serial.println(hue);

  if ((hue > 0.75 || hue <= 0.15)) {  // Red range adjusted to cover around 0.97 and wrap from 1 to 0
    return 'R';
  } else if (hue > 0.15 && hue <= 0.33) {  // Green range now centered at 0.25 with a width of 0.2
    return 'G';
  } else if (hue > 0.33 && hue <= 0.75) {  // Blue range adjusted to fit without overlapping
    return 'B';
  } else {
    return 'U';  // This will now only be for unexpected values outside the range [0,1]
  }




  return hue;
}


int Rcount = 0;
int Gcount = 0;
int Bcount = 0;

//AccelStepper accelP1 = AccelStepper(AccelStepper::DRIVER, motorPin1_P1, motorPin2_P1);

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

  //Serial.println(theta1Deg);

  moveXYWithAbsoluteCoordination(theta1Deg, theta2Deg, myspeed, myspeed / 1.3);
}

void moveToXYYY(float X, float Z, int myspeed, float YYY) {
  const float radToDeg = 180.0 / PI;

  float x = static_cast<float>(X);
  float z = static_cast<float>(Z);

  const float pi = 3.14159265;

  float theta2 = acos((x * x + z * z - L1 * L1 - L2 * L2) / (2 * L1 * L2));
  float theta1 = atan2(z, x) - atan2((L2 * sin(2 * pi - theta2)), (L1 + L2 * cos(2 * pi - theta2)));

  float theta1Deg = theta1 * radToDeg;
  float theta2Deg = theta2 * radToDeg;

  //Serial.println(theta1Deg);

  moveXYWithAbsoluteCoordinationYYY(theta1Deg, theta2Deg, myspeed, myspeed / 1.3, YYY);
}

void moveP1(float Y) {
  float y = Y * 200 / 5 / 3;
  stepper_P1.moveToPositionInSteps(y);
}

Servo Gripper;
const int gripperOpenPos = 60;
const int gripperClosedPos = 180;

const int GripperPin = 5;

int speedAccel = 6000;

void GripperOpen() {
  Gripper.write(gripperOpenPos);
}

void GripperClose() {
  Gripper.write(gripperClosedPos);
}

void setup() {
  Serial.begin(9600);
  pinMode(buttonRpin, INPUT_PULLUP);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1)
      ;
  }
  tcs.setInterrupt(false);

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

  delay(1000);

  Gripper.attach(GripperPin);

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

  stepper_P1.setSpeedInStepsPerSecond(realP1Speed);
  stepper_P1.setAccelerationInStepsPerSecondPerSecond(realP1Accel);

  const float maxHomingDistanceInMM = 20000;

  stepper_RA1.moveToHomeInSteps(1, 800, maxHomingDistanceInMM, 23);
  stepper_RA2.moveToHomeInSteps(-1, 800, maxHomingDistanceInMM, 33);

  moveToXY(80.0, 50.0, 1000);

  stepper_P1.moveToHomeInSteps(-1, 1000, 10000, 27);

  moveP1(31.0);

  pinMode(motorPin1_P1, OUTPUT);
  pinMode(motorPin2_P1, OUTPUT);

  delay(500);

  GripperOpen();

  while (digitalRead(buttonRpin) == 1) {}

  float blockLocations[9][2] = {
    { 205.2, 66.2 },
    { 205.2, 40.8 },
    { 205.2, 15.4 + 6.0 },
    { 270.6, 66.2 },
    { 270.6, 40.8 },
    { 270.6, 15.4 + 6.0 },
    { 336, 66.2 },
    { 336, 40.8 },
    { 336, 15.4 + 6.0 }
  };

  const float redCoordinates[3][3] = {
    { 212.0, 262.3, 200.0 - 5.0 },
    { 212.0, 262.3, 110.0 - 5.0 },
    { 212.0, 262.3, 20.0 - 5.0 }
  };

  const float greenCoordinates[3][3] = {
    { 212.0, 133.52, 200.0 - 5.0 },
    { 212.0, 133.52, 110.0 - 5.0 },
    { 212.0, 133.52, 20.0 - 5.0 }
  };

  const float blueCoordinates[3][3] = {
    { 212.0, 390.78, 200.0 - 5.0 },
    { 212.0, 390.78, 110.0 - 5.0 },
    { 212.0, 390.78, 20.0 - 5.0 }
  };

  int goSpeed = 7000;

  float p1offset = 145.0;

  float greenoffset = 125.0;
  float redoffset = 175.0;
  float blueoffset = 235.0;

  float Xsafe = 80.0;

  int gripperOpenDelay = 350;

  int globalDelay = 10;

  for (int i = 0; i < 9; i++) {
    GripperOpen();
    moveToXY(blockLocations[i][0] - 30.0, blockLocations[i][1], goSpeed);
    delay(globalDelay);
    GripperClose();
    delay(gripperOpenDelay);
    moveToXY(blockLocations[i][0] - 30.0, blockLocations[i][1] + 30.0, goSpeed);
    moveToXY(100.0, 50.0, goSpeed);
    char currentColor = detectColor();
    //while (digitalRead(buttonRpin) == 1) {}
    // go to grid

    Serial.println(currentColor);

    if (currentColor == 'R') {
      moveToXYYY(redCoordinates[Rcount][0] - Xsafe, redCoordinates[Rcount][2], goSpeed, redCoordinates[Rcount][1] + redoffset);
      //moveP1(redCoordinates[Rcount][1] + redoffset);
      delay(globalDelay);
      //moveToXY(redCoordinates[Rcount][0] - Xsafe, redCoordinates[Rcount][2], goSpeed);
      delay(globalDelay);
      moveToXY(redCoordinates[Rcount][0], redCoordinates[Rcount][2], goSpeed);
      GripperOpen();
      delay(gripperOpenDelay);
      moveToXY(redCoordinates[Rcount][0] - Xsafe, redCoordinates[Rcount][2], goSpeed);
      Rcount++;
    } else if (currentColor == 'G') {
      moveToXYYY(greenCoordinates[Gcount][0] - Xsafe, greenCoordinates[Gcount][2], goSpeed, greenCoordinates[Gcount][1] + greenoffset);
      //moveP1(greenCoordinates[Gcount][1] + greenoffset);
      delay(globalDelay);
      //moveToXY(greenCoordinates[Gcount][0] - Xsafe, greenCoordinates[Gcount][2], goSpeed);
      delay(globalDelay);
      moveToXY(greenCoordinates[Gcount][0], greenCoordinates[Gcount][2], goSpeed);
      GripperOpen();
      delay(gripperOpenDelay);
      moveToXY(greenCoordinates[Gcount][0] - Xsafe, greenCoordinates[Gcount][2], goSpeed);
      Gcount++;
    } else if (currentColor == 'B') {
      moveToXYYY(blueCoordinates[Bcount][0] - Xsafe, blueCoordinates[Bcount][2], goSpeed, blueCoordinates[Bcount][1] + blueoffset);
      //moveP1(blueCoordinates[Bcount][1] + blueoffset);
      delay(globalDelay);
      //moveToXY(blueCoordinates[Bcount][0] - Xsafe, blueCoordinates[Bcount][2], goSpeed);
      delay(globalDelay);
      moveToXY(blueCoordinates[Bcount][0], blueCoordinates[Bcount][2], goSpeed);
      GripperOpen();
      delay(gripperOpenDelay);
      moveToXY(blueCoordinates[Bcount][0] - Xsafe, blueCoordinates[Bcount][2], goSpeed);
      Bcount++;
    }

    delay(globalDelay);

    if (i == 8) {
      delay(8000);
    }
    moveToXYYY(130.0, 66.2, goSpeed, 31.0);
  }
}

float curX = 95.174;
float curZ = 271.962;

void loop() {
  char currentColor = detectColor();
  Serial.println(currentColor);
}

float currentAngle1 = 100.0;
float currentAngle2 = 70.0;

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

  //Serial.println(moveAngle1);

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

  //Serial.println(moveSteps1);

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
}













void moveXYWithAbsoluteCoordinationYYY(float targetPosition1, float targetPosition2, float speedInStepsPerSecond, float accelerationInStepsPerSecondPerSecond, float YYY) {
  float y = YYY * 200 / 5 / 3;

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

  //Serial.println(moveAngle1);

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

  //Serial.println(moveSteps1);

  stepper_RA1.setSpeedInStepsPerSecond(speedInStepsPerSecond_1);
  stepper_RA1.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_1);
  stepper_RA1.setupRelativeMoveInSteps(moveSteps1);

  stepper_RA2.setSpeedInStepsPerSecond(speedInStepsPerSecond_2);
  stepper_RA2.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_2);
  stepper_RA2.setupRelativeMoveInSteps(moveSteps2);

  stepper_P1.setSpeedInStepsPerSecond(realP1Speed);
  stepper_P1.setAccelerationInStepsPerSecondPerSecond(realP1Accel);

  stepper_P1.setupMoveInSteps(y);

  while ((!stepper_RA1.motionComplete()) || (!stepper_RA2.motionComplete()) || (!stepper_P1.motionComplete())) {
    stepper_RA1.processMovement();
    stepper_RA2.processMovement();
    stepper_P1.processMovement();
  }

  currentAngle1 += moveAngle1;
  currentAngle2 += moveAngle2;
}
