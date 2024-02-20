#include <AccelStepper.h>
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

/* Defines Motor Pins for Wiring */
const int P1motorPin1 = 2;
const int P1motorPin2 = 3;
const int RA1motorPin1 = 4;
const int RA1motorPin2 = 5;
const int RA2motorPin1 = 6;
const int RA2motorPin2 = 7;

const int GripperPin = 9;

const int joyLXpin = 0;
const int joyLYpin = 1;
const int joyRXpin = 2;
const int joyRYpin = 3;

int joyLX = 0;
int joyLY = 0;
int joyRX = 0;
int joyRY = 0;

const int buttonLpin = 10;
const int buttonRpin = 11;

int buttonL = 0;
int buttonR = 0;

/* Defining Axis Motors via AccelStepper */
AccelStepper P1motor(AccelStepper::DRIVER, P1motorPin1, P1motorPin2);
AccelStepper RA1motor(AccelStepper::DRIVER, RA1motorPin1, RA1motorPin2);
AccelStepper RA2motor(AccelStepper::DRIVER, RA2motorPin1, RA2motorPin2);

/* Gripper Motor Definitions */
Servo Gripper;
const int gripperOpenPos = 0;
const int gripperClosedPos = 180;

void GripperOpen() {
    Gripper.write(gripperOpenPos);
}

void GripperClose() { 
    Gripper.write(gripperClosedPos);
}

/* Axis Class Definition */
class Axis {
    AccelStepper* motor;
    int position;
    int speed;
    int acceleration;

public:
    Axis(AccelStepper* m) : motor(m), position(0), speed(0), acceleration(0) {}

    /* Position Method */
    Axis& setPosition(int pos) {
        position = pos;
        motor->moveTo(position);
        return *this;
    }

    /* Speed Method */
    Axis& setSpeed(int spd) {
        speed = spd;
        motor->setSpeed(speed);
        return *this;
    }

    /* Acceleration Method */
    Axis& setAcceleration(int accel) {
        acceleration = accel;
        motor->setAcceleration(acceleration);
        return *this;
    }

    /* Runs Motor Through AccelStepper */
    void run() {
        motor->run();
    }
};

/* Definition of Each Axis */
Axis P1(&P1motor);
Axis RA1(&RA1motor);
Axis RA2(&RA2motor);

void setup() {
  Serial.begin(9600);

  Gripper.attach(GripperPin);
  system.setAcceleration(100).setSpeed(100);

  pinMode(buttonLpin, INPUT_PULLUP);
  pinMode(buttonRpin, INPUT_PULLUP);
}

void loop() {
    // read all the inputs
    joyLX = analogRead(joyLXpin);
    joyLY = analogRead(joyLYpin);
    joyRX = analogRead(joyRXpin);
    joyRY = analogRead(joyRYpin);

    buttonL = digitalRead(buttonLpin);
    buttonR = digitalRead(buttonRpin);
    
    String output = "LX: " + String(joyLX) + ", LY: " + String(joyLY) +
        ", RX: " + String(joyRX) + ", RY: " + String(joyRY) +
        ", ButtonL: " + String(buttonL) + ", ButtonR: " + String(buttonR);
    Serial.println(output);

    // open gripper based on button presses
    if (buttonL == 1) {
        GripperOpen();
    } else if (buttonR == 1) {
        GripperClose();
    }

    // map joysticks based on anlog input
    int P1speed = map(joyLX, 0, 1023, -255, 255);
    int RA1speed = map(joyLY, 0, 1023, -255, 255);
    int RA2speed = map(joyRY, 0, 1023, -255, 255);

    // set the speed of the motors
    P1motor.setSpeed(P1speed);
    RA1motor.setSpeed(RA1speed);
    RA2motor.setSpeed(RA2speed);

    // motors go brrr
    P1motor.runSpeed();
    RA1motor.runSpeed();
    RA2motor.runSpeed();
}