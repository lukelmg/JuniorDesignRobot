#include <SpeedyStepper.h>

// Define the stepper motor connections for RA1
#define motorPin1_RA1 12
#define motorPin2_RA1 11
// Define the stepper motor connections for RA2
#define motorPin1_RA2 10
#define motorPin2_RA2 9

#define motorPin1_P1 7
#define motorPin2_P1 6

// Initialize the SpeedyStepper objects for both steppers
SpeedyStepper stepper_RA1;
SpeedyStepper stepper_RA2;
SpeedyStepper stepper_P1;

const float L1 = 200.0;
const float L2 = 150.0;

void moveToXY(float X, float Z) {
  const float radToDeg = 180.0 / PI;

  float x = static_cast<float>(X);
  float z = static_cast<float>(Z);

  const float pi = 3.14159265;

  float theta2 = acos((x * x + z * z - L1 * L1 - L2 * L2) / (2 * L1 * L2));
  float theta1 = atan2(z , x) - atan2((L2 * sin(2*pi - theta2)) , (L1 + L2 * cos(2*pi - theta2)));

  float theta1Deg = theta1 * radToDeg;
  float theta2Deg = theta2 * radToDeg;

  /*
  Serial.print("Theta1 (degrees): ");
  Serial.println(theta1Deg);
  Serial.print("Theta2 (degrees): ");
  Serial.println(theta2Deg);
  */

  moveXYWithAbsoluteCoordination(theta1Deg, theta2Deg, 1000, 1000);
  //moveToPosition(stepsTheta1, stepsTheta2);
  delay(500);

}

void moveToPosition(int steps_RA1, int steps_RA2) {
  stepper_RA1.moveToPositionInSteps(steps_RA1);
  stepper_RA2.moveToPositionInSteps(steps_RA2);
}


void setup() {
  Serial.begin(9600);

  // Connect and configure the stepper motors to their IO pins
  stepper_RA1.connectToPins(motorPin1_RA1, motorPin2_RA1);
  stepper_RA2.connectToPins(motorPin1_RA2, motorPin2_RA2);
  stepper_P1.connectToPins(motorPin1_P1, motorPin2_P1);

  pinMode(23, INPUT_PULLUP);
  pinMode(27, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);

  int speedAccel = 2000;

  // Set the speed and acceleration rates for both steppers
  stepper_RA1.setSpeedInStepsPerSecond(speedAccel);
  stepper_RA1.setAccelerationInStepsPerSecondPerSecond(speedAccel);
  stepper_RA2.setSpeedInStepsPerSecond(speedAccel);
  stepper_RA2.setAccelerationInStepsPerSecondPerSecond(speedAccel);

  stepper_P1.setSpeedInStepsPerSecond(1000);
  stepper_P1.setAccelerationInStepsPerSecondPerSecond(5000);

  delay(3000); // Delay to ensure Serial monitor is ready

  const float homingSpeedInMMPerSec = 100;
  const float maxHomingDistanceInMM = 20000;

  
  stepper_RA1.moveToHomeInSteps(1, homingSpeedInMMPerSec, maxHomingDistanceInMM, 23);
  stepper_RA2.moveToHomeInSteps(-1, homingSpeedInMMPerSec, maxHomingDistanceInMM, 33);
  //stepper_P1.moveToHomeInSteps(1, 500, 10000, 27);
  

  delay(1000);

  //moveToXY(95.174, 271.962); //home position

  //stepper_P1.moveToPositionInSteps(-20000);

  moveToXY(100.0, 55.0);
  moveToXY(335.0, 55.0);
  moveToXY(100.0, 55.0);
  moveToXY(335.0, 55.0);
  moveToXY(100.0, 55.0);
}

void loop() {
}

float currentAngle1 = 100.0;
float currentAngle2 = 70.0;

int count = 0;

void moveXYWithAbsoluteCoordination(float targetPosition1, float targetPosition2, float speedInStepsPerSecond, float accelerationInStepsPerSecondPerSecond)
{
  const float stepsPerRevolution = 200.0*4.0*4.0; // 200 steps per rev, 4x gear ratio, 4x microstepping
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

  Serial.println(realMoveAngle2);
  Serial.println(moveAngle1);
  Serial.println(moveAngle2);
  
  //moveAngle2 = currentAngle2 - targetPosition2 + moveAngle1; // first pos up
  //moveAngle2 = currentAngle2 + targetPosition2 - moveAngle1; // ra2 down way to far on first pos
  //moveAngle2 = currentAngle2 + targetPosition2 + moveAngle1; // down too far
  //moveAngle2 = -currentAngle2 + targetPosition2 + moveAngle1; // up far
  //moveAngle2 = - currentAngle2 + targetPosition2 - moveAngle1; // KINDA CLOSE??? FIRST POS IS TOO LOW
  //moveAngle2 = -currentAngle2 - targetPosition2 - moveAngle1; //uppies
  //moveAngle2 = -currentAngle2 - targetPosition2 +  moveAngle1; // smack into ground

  //Serial.println("NEW");

/*
  Serial.print("Target Position 1: ");
  Serial.print(targetPosition1);
  Serial.print(", Current Angle 1: ");
  Serial.print(currentAngle1);
  Serial.print(", Move Angle 1: ");
  Serial.print(moveAngle1);
  Serial.print(", Target Position 2: ");
  Serial.print(targetPosition2);
  Serial.print(", Current Angle 2: ");
  Serial.print(currentAngle2);
  Serial.print(", Move Angle 2: ");
  Serial.println(moveAngle2); // Ends the line here
*/

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

   if ((absSteps1 > absSteps2) && (moveSteps1 != 0))
  {
    //
    // slow down the motor traveling less far
    //
    float scaler = (float) absSteps2 / (float) absSteps1;
    speedInStepsPerSecond_2 = speedInStepsPerSecond_2 * scaler;
    accelerationInStepsPerSecondPerSecond_2 = accelerationInStepsPerSecondPerSecond_2 * scaler;
  }
  
  if ((absSteps2 > absSteps1) && (moveSteps2 != 0))
  {
    //
    // slow down the motor traveling less far
    //
    float scaler = (float) absSteps1 / (float) absSteps2;
    speedInStepsPerSecond_1 = speedInStepsPerSecond_1 * scaler;
    accelerationInStepsPerSecondPerSecond_1 = accelerationInStepsPerSecondPerSecond_1 * scaler;
  }

  stepper_RA1.setSpeedInStepsPerSecond(speedInStepsPerSecond_1);
  stepper_RA1.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_1);
  stepper_RA1.setupRelativeMoveInSteps(moveSteps1); // Hypothetical function for absolute moves

  stepper_RA2.setSpeedInStepsPerSecond(speedInStepsPerSecond_2);
  stepper_RA2.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_2);
  stepper_RA2.setupRelativeMoveInSteps(moveSteps2); // Hypothetical function for absolute moves

  //
  // now execute the moves, looping until both motors have finished
  //
  while((!stepper_RA1.motionComplete()) || (!stepper_RA2.motionComplete()))
  {
    stepper_RA1.processMovement();
    stepper_RA2.processMovement();
  }

  currentAngle1 += moveAngle1;
  currentAngle2 += moveAngle2;
  count++;
}
