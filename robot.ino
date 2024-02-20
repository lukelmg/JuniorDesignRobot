#include <AccelStepper.h>
#include <Servo.h>


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

class Axis {
    AccelStepper* motor;
    int position;
    int speed;
    int acceleration;

public:
    Axis(AccelStepper* m) : motor(m), position(0), speed(0), acceleration(0) {}

    Axis& setPosition(int pos) {
        position = pos;
        motor->moveTo(position);
        return *this;
    }

    Axis& setSpeed(int spd) {
        speed = spd;
        motor->setSpeed(speed);
        return *this;
    }

    Axis& setAcceleration(int accel) {
        acceleration = accel;
        motor->setAcceleration(acceleration);
        return *this;
    }

    void run() {
        motor->run();
    }
};

Axis P1(&P1motor);
Axis RA1(&RA1motor);
Axis RA2(&RA2motor);

class GoToPosition {
    Axis *P1, *RA1, *RA2;

public:
    GoToPosition(Axis *p1, Axis *ra1, Axis *ra2)
        : P1(p1), RA1(ra1), RA2(ra2) {}

    GoToPosition& moveTo(int X, int Y, int Z) {
        P1->setPosition(X);
        RA1->setPosition(Y);
        RA2->setPosition(Z);
        return *this;
    }

    GoToPosition& setAcceleration(int accel) {
        P1->setAcceleration(accel);
        RA1->setAcceleration(accel);
        RA2->setAcceleration(accel);
        return *this;
    }

    GoToPosition& setSpeed(int speed) {
        P1->setSpeed(speed);
        RA1->setSpeed(speed);
        RA2->setSpeed(speed);
        return *this;
    }

    void run() {
        P1->run();
        RA1->run();
        RA2->run();
    }
};

GoToPosition system(&P1, &RA1, &RA2);

const float pickupCoordinates[9][3] = {
  {100, 200, 300}, {150, 250, 350}, {200, 300, 400},
  {250, 350, 450}, {300, 400, 500}, {350, 450, 550},
  {400, 500, 600}, {450, 550, 650}, {500, 600, 700}
};

const float gridCoordinates[9][3] = {
  {100, 200, 300}, {150, 250, 350}, {200, 300, 400},
  {250, 350, 450}, {300, 400, 500}, {350, 450, 550},
  {400, 500, 600}, {450, 550, 650}, {500, 600, 700}
};

int safeXPosition = 100;

int currentCoordinate = 0;

void setup() {
  Gripper.attach(GripperPin);
  system.setAcceleration(100).setSpeed(100);
}

void loop() {
  if (currentCoordinate < 9) {
    gripperOpen();

    delay(1000);

    system.moveTo(pickupCoordinates[currentCoordinate][0], pickupCoordinates[currentCoordinate][1], pickupCoordinates[currentCoordinate][2]);
    system.run();

    delay(1000);

    gripperClose();

    delay(1000);

    system.moveTo(safeXPosition, pickupCoordinates[currentCoordinate][1], pickupCoordinates[currentCoordinate][2]);
    system.run();

    delay(1000);

    currentCoordinate++;
  }
}