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

    GoToPosition& moveTo(float X, int Y, float Z) {
        const float L1 = 200.0;
        const float L2 = 150.0;

        const int pitch = 5;
        const int stepsPerRevolution = 200;

        float x = static_cast<float>(X);
        float z = static_cast<float>(Z);

        float theta1 = atan2(z, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
        float theta2 = acos((x*x + z*z - L1*L1 - L2*L2) / (2 * L1 * L2));

        int invP1 = stepsPerRevolution * Y / pitch;
        int invRA1 = static_cast<int>(theta1);
        int invRA2 = static_cast<int>(theta2);

        P1->setPosition(invP1);
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

GoToPosition Robot(&P1, &RA1, &RA2);

/* Pickup Positions at Each Table Position */
const float pickupCoordinates[9][3] = {
  {205.20, 0.00, 66.20}, {205.20, 0.00, 40.80}, {205.20, 0.00, 15.40},
  {270.60, 0.00, 66.20}, {270.60, 0.00, 40.80}, {270.60, 0.00, 15.40},
  {336.00, 0.00, 66.20}, {336.00, 0.00, 40.80}, {336.00, 0.00, 15.40}
};

/* Dropoff Positions at Each Grid Position */
const float redCoordinates[3][3] = {
  {212.0, 262.3, 200.0}, {212.0, 262.3, 110.0}, {212.0, 262.3, 20.0}
};

const float greenCoordinates[3][3] = {
  {212.0, 133.52, 200.0}, {212.0, 133.52, 110.0}, {212.0, 133.52, 110.0}
};

const float blueCoordinates[3][3] = {
  {212.0, 390.78, 200.0}, {212.0, 390.78, 110.0}, {212.0, 390.78, 20.0}
};

// Safe retracted back position in X direction
int safeXPosition = 100;

int currentCoordinate = 0;

int Rcount = 0;
int Gcount = 0;
int Bcount = 0;

void setup() {
  Serial.begin(9600);

  Gripper.attach(GripperPin);
  Robot.setAcceleration(100).setSpeed(100);

  /* Check for Color Sensor */
  if (tcs.begin()) {
    Serial.println("Sensor found");
  } else {
    Serial.println("No sensor found");
    while (1);
  }
}

void loop() {
  if (currentCoordinate < 9) {
    // Open Gripper
    GripperOpen();

    // Go to Table Position
    Robot.moveTo(pickupCoordinates[currentCoordinate][0], pickupCoordinates[currentCoordinate][1], pickupCoordinates[currentCoordinate][2]);
    Robot.run();

    // Grab Cube
    GripperClose();

    // Detect Color
    char currentColor = readColorSensor();

    // Go to Table Safe Position
    Robot.moveTo(safeXPosition, pickupCoordinates[currentCoordinate][1], pickupCoordinates[currentCoordinate][2] + 20);
    Robot.run();

    // Based on color, go to different grid dropoff positions
    if (currentColor == 'R') {
        Robot.moveTo(safeXPosition, redCoordinates[Rcount][1], redCoordinates[Rcount][2]);
        Robot.run();
        Robot.moveTo(safeXPosition+100.0, redCoordinates[Rcount][1], redCoordinates[Rcount][2]);
        Robot.run();
        Rcount++;
    } else if (currentColor == 'G') {
        Robot.moveTo(safeXPosition, greenCoordinates[Gcount][1], greenCoordinates[Gcount][2]);
        Robot.run();
        Robot.moveTo(safeXPosition+100.0, greenCoordinates[Gcount][1], greenCoordinates[Gcount][2]);
        Robot.run();
        Gcount++;
    } else if (currentColor == 'B') {
        Robot.moveTo(safeXPosition, blueCoordinates[Bcount][1], blueCoordinates[Bcount][2]);
        Robot.run();
        Robot.moveTo(safeXPosition+100.0, blueCoordinates[Bcount][1], blueCoordinates[Bcount][2]);
        Robot.run();
        Bcount++;
    }

    GripperOpen();

    currentCoordinate++;
  }
}