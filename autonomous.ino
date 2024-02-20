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

/* Detect Color Function*/
const char* readColorSensor() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  if (r > g && r > b) {
    return "R";
  } else if (g > r && g > b) {
    return "G";
  } else if (b > r && b > g) {
    return "B";
  } else {
    return "U";
  }
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

/* Inverse Kinematics Go To Motor Position Function */
class GoToPosition {
    Axis *P1, *RA1, *RA2;

public:
    GoToPosition(Axis *p1, Axis *ra1, Axis *ra2)
        : P1(p1), RA1(ra1), RA2(ra2) {}

    GoToPosition& moveTo(int X, int Y, int Z) {
        // insert inverse kinematics equation here
        int invP1 = X;
        int invRA1 = Y;
        int invRA2 = Z;

        P1->setPosition(invP1);
        RA1->setPosition(invRA1);
        RA2->setPosition(invRA2);
        return *this;
    }

    GoToPosition& moveX(int X) {
        // insert inverse kinematics equation here
        int invRA1 = X;
        int invRA2 = Y;

        RA1->setPosition(invRA1);
        RA2->setPosition(invRA2);
        return *this;
    }

    /* Transferring of Axis Acceleration Method */
    GoToPosition& setAcceleration(int accel) {
        P1->setAcceleration(accel);
        RA1->setAcceleration(accel);
        RA2->setAcceleration(accel);
        return *this;
    }

    /* Transferring of Axis Speed Method */
    GoToPosition& setSpeed(int speed) {
        P1->setSpeed(speed);
        RA1->setSpeed(speed);
        RA2->setSpeed(speed);
        return *this;
    }

    /* Runs Each Axis Through AccelStepper */
    void run() {
        P1->run();
        RA1->run();
        RA2->run();
    }
};

GoToPosition system(&P1, &RA1, &RA2);

/* Pickup Positions at Each Table Position */
const float pickupCoordinates[9][3] = {
  {100, 100, 100}, {100, 100, 100}, {100, 100, 100},
  {100, 100, 100}, {100, 100, 100}, {100, 100, 100},
  {100, 100, 100}, {100, 100, 100}, {100, 100, 100}
};

/* Dropoff Positions at Each Grid Position */
const float redCoordinates[3][3] = {
  {100, 100, 100}, {100, 100, 100}, {100, 100, 100}
}

const float greenCoordinates[3][3] = {
  {100, 100, 100}, {100, 100, 100}, {100, 100, 100}
}

const float blueCoordinates[3][3] = {
  {100, 100, 100}, {100, 100, 100}, {100, 100, 100}
}

// Safe retracted back position in X direction
int safeXPosition = 100;

int currentCoordinate = 0;

int Rcount = 0;
int Gcount = 0;
int Bcount = 0;

void setup() {
  Serial.begin(9600);

  Gripper.attach(GripperPin);
  system.setAcceleration(100).setSpeed(100);

  /* Check for Color Sensor */
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
}

void loop() {
  if (currentCoordinate < 9) {
    // Open Gripper
    gripperOpen();

    // Go to Table Position
    system.moveTo(pickupCoordinates[currentCoordinate][0], pickupCoordinates[currentCoordinate][1], pickupCoordinates[currentCoordinate][2]);
    system.run();

    // Grab Cube
    gripperClose();

    // Detect Color
    char currentColor = readColorSensor();

    // Go to Table Safe Position
    system.moveTo(safeXPosition, pickupCoordinates[currentCoordinate][1], pickupCoordinates[currentCoordinate][2] + 20);
    system.run();

    // Based on color, go to different grid dropoff positions
    if (currentColor == 'R') {
        system.moveTo(safeXPosition, redCoordinates[Rcount][1], redCoordinates[Rcount][2]);
        system.run();
        system.moveX(100);
        system.run();
        Rcount++;
    } else if (currentColor == 'G') {
        system.moveTo(safeXPosition, greenCoordinates[Gcount][1], greenCoordinates[Gcount][2]);
        system.run();
        system.moveX(100);
        system.run();
        Gcount++;
    } else if (currentColor == 'B') {
        system.moveTo(safeXPosition, blueCoordinates[Bcount][1], blueCoordinates[Bcount][2]);
        system.run();
        system.moveX(100);
        system.run();
        Bcount++;
    }

    gripperOpen();

    system.moveX(50);

    currentCoordinate++;
  }
}