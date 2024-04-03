#include <AccelStepper.h>
#include <SpeedyStepper.h>

// Define step and direction interface pins
#define stepPin 7
#define dirPin  6

// Initialize AccelStepper for a motor using a driver (STEP and DIR)
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

void setup() {
  // Set the maximum speed in steps per second:
  stepper.setMaxSpeed(1000);
  
  // Set the desired speed in steps per second:
  stepper.setSpeed(500);
}

void loop() {
  // Move the motor with a constant speed as set by setSpeed():
  stepper.runSpeed();
}
