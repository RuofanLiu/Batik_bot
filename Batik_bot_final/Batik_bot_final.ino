#include<DFRobotHighTemperatureSensor.h>
#include <AccelStepper.h>                     // V1.30, For running motors simultaneously
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <avr/pgmspace.h>                     // Store data in flash (program) memory instead of SRAM

#define runningSpeed 200
/* This program was created by ScottC on 8/5/2012 to receive serial
  signals from a computer to turn on/off 1-9 LEDs */

//----------------------other variables--------------------//
int isFirstValue = 0;
int maxScale; //this variable is used to determine the scale of the painting on the turning table.
String val;
unsigned long center = 0; //this variable stoers the number of steps needed for the syringe to reach the center of the turning table
//----------------------other variables--------------------//



//-------------------for Hall effect sensor----------------//
const int hallTurningTable = 2;
const int hallLeadScrewRight = 3;
const int hallLeadScrewLeft = 4;
const int hallLSyringe = 5;
int hallStateTT = 0;
int hallStateLSright = 0;
int hallStateLSleft = 0;
int hallStateS = 0;

//-------------------for Hall effect sensor----------------//




//-----------------for Temperature sensor------------------//
const float voltageRef = 5.000; //Set reference voltage,you need test your IOREF voltage.
//const float voltageRef = 3.300;
int HighTemperaturePin = A0;  //Setting pin
DFRobotHighTemperature PT100 = DFRobotHighTemperature(voltageRef); //Define an PT100 object
//-----------------for Temperature sensor------------------//



//------------------for stepper motor---------------------//
//
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
Adafruit_MotorShield AFMS_level2 = Adafruit_MotorShield(0x61);

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4), port #1 is M1 and M2
Adafruit_StepperMotor *turningTable = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *leadScrew = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *syringe = AFMS_level2.getStepper(200, 1);

Adafruit_StepperMotor *myHeater = AFMS_level2.getStepper(200, 1); //for the heating resistor
//------------------for stepper motor---------------------//


// Reset function for program restart in case of failure
void(* resetFunc) (void) = 0;//declare reset function at address 0

/* Create wrapper functions for use with the AccelStepper Library
   Can change to MICROSTEP for more accuracy, or DOUBLE for more torque
   Angular Motor
   These functions are from https://github.com/SDMelee/BatikBot
*/
void forwardstepA() {
  turningTable->onestep(FORWARD, SINGLE);
}
void backwardstepA() {
  turningTable->onestep(BACKWARD, SINGLE);
}
// Radial Motor
void forwardstepR() {
  leadScrew->onestep(FORWARD, SINGLE);
}
void backwardstepR() {
  leadScrew->onestep(BACKWARD, SINGLE);
}
// Glue  Gun Press Motor
void forwardstepE() {
  syringe->onestep(FORWARD, DOUBLE);
}
void backwardstepE()  {
  syringe->onestep(BACKWARD, DOUBLE);
}
// Create Astepper and Rstepper objects from AccelStepper library
AccelStepper Astepper(forwardstepA, backwardstepA); // use functions to step  for turning table
AccelStepper Rstepper(forwardstepR, backwardstepR); // use functions to step  for leadscrew
AccelStepper Estepper(forwardstepE, backwardstepE); // use functions to step  for syringe


/**
   This function returns the temperature measured by the temperature sensor
*/
int readTemp() {
  return PT100.readTemperature(HighTemperaturePin);
}

/**
   This function initialize the position of the lead screw and determines the center of the turning table
   (this worked, but kinda buggy interms of the polar of the magnet issue (reset))
*/
void initializeScrew() {
  /*
     Use timer to help calculating the center
  */
  unsigned long startTime = 0;
  unsigned long endTime = 0;
  int steps = 0;
  int reachedRightmost = 0;
  int reachedLeftmost = 0;
  unsigned long duration = 0; //in seconds

  pinMode(hallLeadScrewRight, INPUT); //digit port 3
  pinMode(hallLeadScrewLeft, INPUT);  //digit port 4
  while (reachedRightmost == 0 && hallStateLSright == LOW) {
    hallStateLSright = digitalRead(hallLeadScrewRight);
    Rstepper.setSpeed(200);
    Rstepper.runSpeed();
  }
  while (reachedRightmost == 0 && hallStateLSright == HIGH) {
    hallStateLSright = digitalRead(hallLeadScrewRight);
    Rstepper.setSpeed(200);
    Rstepper.runSpeed();
  }
  reachedRightmost = 1;
  delay(2000);

  if (hallStateLSright == HIGH) {
    startTime = millis();
  }
  while (reachedRightmost == 1 && hallStateLSleft == LOW) {
    hallStateLSleft = digitalRead(hallLeadScrewLeft);
    Rstepper.setSpeed(-200);
    Rstepper.runSpeed();
  }

  if (hallStateLSleft == HIGH) {
    endTime = millis();
  }
  delay(5000);
  /*Calculate the number of steps needed*/
  //this doesnt work
  duration = (endTime - startTime) / 1000;
  Serial.print("Start time: ");
  Serial.println(startTime);
  Serial.print("End time: ");
  Serial.println(endTime);
  Serial.print("duration ");
  Serial.println(duration);
  center = (200 * duration) / 2;
  //finally move the syringe to the center of the turning table, begin to draw

  Rstepper.setCurrentPosition(0);
  Rstepper.moveTo(center);
  Serial.print("Steps to center: ");
  Serial.println(center);
  while (Rstepper.currentPosition() != Rstepper.targetPosition()) {
    Rstepper.setSpeed(200);
    Rstepper.runSpeedToPosition();
  }
  reachedLeftmost = 1;
}

/**
   This function initialize the position of the turning table and the leadscrew and syringe
   should be called in setup()
*/
void initializeTable() {
  /*
     first check the current position of the turning table.
     If it is not at the initial position, continue turning
  */
  pinMode(hallTurningTable, INPUT);

  while (hallStateTT == LOW) {
    hallStateTT = digitalRead(hallTurningTable);
    Astepper.setSpeed(100);
    Astepper.run();
  }
  /*
     since the US1881 hall sensor is blocked, move one step back to re-set its value to low
  */
  while (hallStateTT == HIGH) {
    hallStateTT = digitalRead(2);
    Astepper.move(1);
    Astepper.run();
  }
}


/**
   This function controls the top stepper motor to apply wax
*/
void applyWax() {
  //Estepper.setSpeed(extruderSpeed);
  Estepper.move(50); // Move 2.5 rotations or 0.5cm down
  // Use non-blocked command in while-loop instead of blocked command
  // because blocked command does not work for some reason...
  Serial.println("Pressing syringe down");
  while (Estepper.currentPosition() != Estepper.targetPosition()) {
    Estepper.runSpeedToPosition();
  }

  Serial.println("Waiting for user input to move to next position!");
  //TO DO: send ACK to Processing code and wait until next value is received
  //DO NOT WORRY ABOUT TODO IN THIS SCOPE FOR NOW
}

/*
   Initialize all the ports
*/
void setup() {
  Serial.begin(9600);
  while (Serial.available() <= 0) {
    Serial.println("ACK\n");   // send COntact to initialize the data transimission
    delay(1000);
  }
  val = Serial.readStringUntil('\n');
  delay(100); //this is necessary, as the arduino CPU is too slow to read data
  Serial.println("ACK\n");

  //------------------for stepper motor---------------------//
  AFMS.begin(1600);  // set the frequencey to 1.6k Hz
  AFMS_level2.begin(1600);

  Rstepper.setMaxSpeed(1000);
  initializeTable();
  delay(2000);
  initializeScrew();
  //myHeater->setSpeed(50);
  //------------------for stepper motor---------------------//
  //  maxScale = val.toInt();
}

void loop() {
  double radii = 0;
  int angle = 0;
  /*
     Serial Communication
     The basic idea is, since the Arduino Uno memory is too small to store all the data pointes, Processing is used to send one set of data at a time
     When a set of data is received, send ACK to Processing so that the next data will be sent
  */
  if (Serial.available() > 0) { // If data is available to read,
    radii = Serial.readStringUntil(',').toDouble();
    angle = Serial.readStringUntil('\n').toInt();

    Serial.print("radii: ");
    Serial.println(radii);  //THIS LINE IS FOR TEST ONLY
    Serial.print("angle: ");
    Serial.println(angle);
    Serial.print("maxScale: ");
    Serial.println(maxScale);
    delay(100);   //this is necessary FOR NOW
    Serial.println("ACK\n");
  }


  //Rstepper.run();  //this works, first accelerates and decellerates

}
