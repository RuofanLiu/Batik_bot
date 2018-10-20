#include<DFRobotHighTemperatureSensor.h>
#include <AccelStepper.h>                     // V1.30, For running motors simultaneously
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <avr/pgmspace.h>                     // Store data in flash (program) memory instead of SRAM
/* This program was created by ScottC on 8/5/2012 to receive serial
  signals from a computer to turn on/off 1-9 LEDs */

//----------------------other variables--------------------//
int maxScale = 0; //this variable is used to determine the scale of the painting on the turning table.
String val;
//----------------------other variables--------------------//

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
Adafruit_StepperMotor *turningTable = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *leadScrew = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *syringe = AFMS_level2.getStepper(200, 1);

Adafruit_StepperMotor *myHeater = AFMS_level2.getStepper(200, 1); //for the heating resistor
//------------------for stepper motor---------------------//


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
 * Initialize all the ports
 */
void setup() {
  Serial.begin(9600);
  //printf("String %s\n", ByteRead);
  while (Serial.available() <= 0) {
    Serial.println("ACK\n");   // send COntact to initialize the data transimission
    delay(500);
  }
  //------------------for stepper motor---------------------//
  AFMS.begin(1600);  // OR with a different frequency, say 1KHz
  AFMS_level2.begin(1600);

  turningTable->setSpeed(50);  // 50 rpm
  leadScrew->setSpeed(200);
  syringe->setSpeed(50);

  //------------------test only------------------//
  Rstepper.setMaxSpeed(200);
  Rstepper.moveTo(1500);
  //------------------test only------------------//

  myHeater->setSpeed(50);
  //------------------for stepper motor---------------------//
}

void loop() {
  /*
   * Serial Communication
   * The basic idea is, since the Arduino Uno memory is too small to store all the data pointes, Processing is used to send one set of data at a time
   * When a set of data is received, send ACK to Processing so that the next data will be sent
   */
  if (Serial.available() > 0) { // If data is available to read,
    val = Serial.readStringUntil('\n'); // read the csv value sent from Processing
    if (val != NULL) {
      Serial.println("ACK\n");
    }
    if (val != "ACK") {     //THIS IF STATEMENT IS FOR TEST ONLY
      Serial.println(val);
    }
  }

  //Rstepper.run();  //this works, first accelerates and decellerates


  //TO DO: now serial communication worked, change the Processing code and check the csv file next
}
