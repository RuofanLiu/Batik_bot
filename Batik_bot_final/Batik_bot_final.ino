/**
   This file is developed by Ruofan Liu on October 27, 2018
   It controlls the batik bot to draw patterns. The user will design a pattern and download the coordinate csv file from
   https://dev.csdt.rpi.edu/applications/56/run
   This program will read the coordinate csv file and use wax to draw the exact same pattern on a cloth.
   For temperature control, PID control is not needed as the temperature rise is really slow
   TO DO: 1. Use Hall-effect sensor to check the position of the syringe so that it wont push until the very end (Estepper)
          2. implement the debug mode
   Many thanks to the following people:
        David Goldschmidt, William Babbit, Mukkai Krishnamoorthy, Ron Eglash, James Davis, Yudan Liu, Abraham Ferraro, Yuxiang Meng, Chenyu Wu, Paween Pitimanaaree,
        for giving me advice and helping me in designing the machine and moving it when needed
*/
#include<DFRobotHighTemperatureSensor.h>
#include <AccelStepper.h>                     // V1.30, For running motors simultaneously
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <avr/pgmspace.h>                     // Store data in flash (program) memory instead of SRAM

#define meltingPoint 52
//----------------------other variables--------------------//
int isFirstValue = 0;
double maxScale; //this variable is used to determine the scale of the painting on the turning table.
String val;
unsigned long center = 0; //this variable stoers the number of steps needed for the syringe to reach the center of the turning table
//THIS VALUE IS NEEDED FOR CALCULATING THE ACTUAL SCALE< DON"T CLEAR IT OUT
int currentTemperature = 0;
int pointReached = 0;   //a boolean variable that indicates whether the arm has reached the designed location
int waxApplied = 1;   //a boolean variable that indicates whether the was has been applied
int startTemp = 0;
int stepToPush = 80;
int period = 0;

//-------------------for Hall effect sensor----------------//
const int hallTurningTable = 2;
const int hallLeadScrewRight = 3;
const int hallLeadScrewLeft = 4;
const int hallLSyringe = 5;
int hallStateTT = 0;
int hallStateLSright = 0;
int hallStateLSleft = 0;
int hallStateS = 0;

//-----------------for Temperature sensor------------------//
const float voltageRef = 5.000; //Set reference voltage,you need test your IOREF voltage.
//const float voltageRef = 3.300;
int HighTemperaturePin = A0;  //Setting pin
DFRobotHighTemperature PT100 = DFRobotHighTemperature(voltageRef); //Define an PT100 object

//------------------for stepper motor---------------------//
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
Adafruit_MotorShield AFMS_level2 = Adafruit_MotorShield(0x61);

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4), port #1 is M1 and M2
Adafruit_StepperMotor *turningTable = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *leadScrew = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *syringe = AFMS_level2.getStepper(200, 2);
Adafruit_StepperMotor *myHeater = AFMS_level2.getStepper(200, 1); //for the heating resistor


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
    This function enables the heating pad to heat up to 65 degree Celsius
*/
void initializeHeater() {
  int counter = 0;
  int previousTemp = 0 ;
  currentTemperature = PT100.readTemperature(HighTemperaturePin);
  //previousTemp = currentTemperature;
  int curtemp = currentTemperature;
  Serial.print("curtemp:  ");
  Serial.println(currentTemperature);
  myHeater->onestep(FORWARD, DOUBLE);
  
  while (currentTemperature < meltingPoint) {
    if(counter == 10000){
      counter = 0;
    }
    if(counter % 2000 == 0){
      currentTemperature = PT100.readTemperature(HighTemperaturePin);  //Get temperature
      Serial.print("temperature:  ");
      Serial.println(currentTemperature);
    }
    if(currentTemperature == meltingPoint && (currentTemperature - previousTemp) > 0 && (currentTemperature - previousTemp) <= 1){
      break;
    }
    previousTemp = currentTemperature;
    counter++;
    myHeater->onestep(FORWARD, DOUBLE);
  }
  //TO DO: try to read the temperature again at this line //not correct, curtemp changes
  //should always start from 47
  Serial.println("delaying...");
  //long waittime = 300000; //295000 technically
  //long actual = waittime*(48 - curtemp)/(48 - 30);
  
  curtemp = PT100.readTemperature(HighTemperaturePin);
//  Serial.print("curtemp:  ");
//  Serial.println(curtemp);
//  long actual = (55 - curtemp)*(unsigned long)32 * 1000;  //it takes 31.3 sec on average to raise up 1 degree, and 55 degree Celcius is the point where the wax is perfect for drawing
//  Serial.print("Delay Time: ");
//  Serial.print(actual);
//  Serial.println("ms");
//  delay(actual);  
//  Serial.println("delay done");
//  delay(45000); //it was 30000 ms
//  myHeater -> release();
  unsigned long startTime = 0;
  unsigned long endTime = 0;
  int counter1 = 0;
  startTime = millis();
  endTime = millis();
  Serial.println("maintaining temperature for 10 minutes");
  while(endTime - startTime < 600000){
    Serial.println(endTime - startTime);
    if(counter1 % 250 == 0){
      currentTemperature = PT100.readTemperature(HighTemperaturePin);
      if(currentTemperature < 53){
        myHeater->onestep(FORWARD, DOUBLE);
      }
      if(currentTemperature > 53){
        myHeater->release();
      }
      if(counter1 ==  250){
        counter1 = 0;
      }
    }
    endTime = millis();
    counter1++;
  }
  
  startTemp = PT100.readTemperature(HighTemperaturePin);
}


/**
   This function initialize the position of the lead screw and determines the center of the turning table
   After this initialization, the syringe will be located to the center of the table with targetPosition = 0
*/
void initializeScrew() {
  /*
     Use timer to help calculating the center
     move to rightmost first, and then start calculating time
     after that, move to left most. use time and spped to calculate the center
     Prerequisite: The Hall-effect sensor and the magnets are setup at proper places
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

  //manually set it to low
  hallStateLSright = digitalRead(hallLeadScrewRight);
  reachedRightmost = 1;
  delay(2000);

  startTime = millis();
  while (reachedRightmost == 1 && hallStateLSleft == LOW) {
    hallStateLSleft = digitalRead(hallLeadScrewLeft);
    Rstepper.setSpeed(-200);
    Rstepper.runSpeed();
  }
  endTime = millis();
  hallStateLSleft = digitalRead(hallLeadScrewLeft);
  delay(3000);
  /*Calculate the number of steps needed*/
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
  Rstepper.setCurrentPosition(0);
  Serial.print("Current location: ");
  Serial.println(Rstepper.currentPosition()); //supposed to print out 0
  //release the stepper motor to save the current
  leadScrew -> release();
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
    Astepper.move(1);
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
  Astepper.setCurrentPosition(0);
  delay(1500);
  //release the stepper motor to save the current
  turningTable -> release();
}

/**
   The following function enables the syringe to find and move to the designated coordinate read from the computer
*/
void moveAndDraw(double radii, double angle) {
  pointReached = 0;
  Astepper.enableOutputs();
  Rstepper.enableOutputs();
  double perUnit = center / maxScale;
  int stepToMove = radii * perUnit;   //for lead screw
  int stepToRotate = angle / 1.8;  //for turning table. The minimum steps the stepper motor can move is 1.8 degree

  Serial.println("All initialized");

  Serial.print("Step to move: "); Serial.println(stepToMove);
  Rstepper.moveTo(stepToMove);
  while (Rstepper.currentPosition() != Rstepper.targetPosition()) {
    if (stepToMove < Rstepper.currentPosition()) {
      Rstepper.setSpeed(-1 * 250);
    }
    else {
      Rstepper.setSpeed(250);
    }

    Rstepper.runSpeedToPosition();
  }
  delay(1000);

  Serial.print("Step to rotate: "); Serial.println(stepToRotate);
  Astepper.moveTo(stepToRotate);
  while (Astepper.currentPosition() != Astepper.targetPosition()) {
    if (stepToRotate < Astepper.currentPosition()) {
      Astepper.setSpeed(-1 * 5);
    }
    else {
      Astepper.setSpeed(5);
    }

    Astepper.runSpeedToPosition();
  }
  delay(1500);
  pointReached = 1;

  //release the stepper motor to save the current
  leadScrew -> release();
  turningTable -> release();
}


/**
   This function controls the top stepper motor to push the syringe and apply wax
*/
void applyWax() {
  waxApplied = 0;
  //Estepper.setSpeed(extruderSpeed);
  Estepper.move(stepToPush); // Move 2.5 rotations or 0.5cm down
  // Use non-blocked command in while-loop instead of blocked command
  // because blocked command does not work for some reason...
  Serial.println("Pressing syringe down");
  while (Estepper.currentPosition() != Estepper.targetPosition()) {
    Estepper.setSpeed(50);
    Estepper.runSpeedToPosition();
  }
  //release the stepper motor to save the current
  delay(1000);
//  Estepper.move(-15); // Move 2.5 rotations or 0.5cm down
//  // Use non-blocked command in while-loop instead of blocked command
//  // because blocked command does not work for some reason...
//  Serial.println("Pressing syringe down");
//  while (Estepper.currentPosition() != Estepper.targetPosition()) {
//    Estepper.setSpeed(-50);
//    Estepper.runSpeedToPosition();
//  }
  syringe -> release();
  //stepToPush += 15; //the offset of having the Estepper move up
  waxApplied = 1;
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
  delay(500); //this is necessary, as the arduino CPU is too slow to read data
  Serial.println("ACK\n");

  Serial.println("Program start. Initializing...");
  //------------------for stepper motor---------------------//
  AFMS.begin(1600);  // set the frequencey to 1.6k Hz
  AFMS_level2.begin(1600);

  //initialize three stepper motors
  Rstepper.setMaxSpeed(1000);
  Astepper.setMaxSpeed(500);
  Estepper.setMaxSpeed(250);

//  Serial.println("Initializing turning table...");
  initializeTable();
  delay(2000);
  Serial.println("Initializing lead screw...");
  initializeScrew();
  delay(3000);
  //read temperature
  Serial.println("Heating up to melt the batik wax...");
  //heat the heating pad up tp 60 degree to melt the wax
  initializeHeater();
  currentTemperature = readTemp();
  Serial.print("Heating up done, current temperature is: ");
  Serial.println(currentTemperature);
  delay(1500);
  maxScale = val.toDouble();
  Serial.println("All Initialization done.");
}

void loop() {
  double radii = 0;
  double angle = 0;
  String temp1;
  String temp2;
  /*
     Serial Communication
     The basic idea is, since the Arduino Uno memory is too small to store all the data pointes, Processing is used to send one set of data at a time
     When a set of data is received, send ACK to Processing so that the next data will be sent
  */
  if (Serial.available() > 0) { // If data is available to read,
    temp1 = Serial.readStringUntil(',');
    temp2 = Serial.readStringUntil('\n');

    //when all the points are drawn
    if (temp1 == "Finished" || temp2 == "Finished") {
      leadScrew -> release();
      syringe -> release();
      myHeater -> release();
      turningTable -> release();
      Astepper.disableOutputs();
      Rstepper.disableOutputs();
      Estepper.disableOutputs();
      delay(1000);  // Delay to allow above message to finish printing to monitor
      Serial.println("DONE\n");
      resetFunc(); //call reset
    }
    else {
      radii = temp1.toDouble();
      angle = temp2.toDouble();
      currentTemperature = readTemp();
      
      //send information to Processing
      Serial.print("radii: ");
      Serial.println(radii);  //THIS LINE IS FOR TEST ONLY
      Serial.print("angle: ");
      Serial.println(angle);
      Serial.print("maxScale: ");
      Serial.println(maxScale);

      if (waxApplied == 1) {
        Serial.println("moving to position");
        moveAndDraw(radii, angle);
        waxApplied = 0;
      }
      if (pointReached == 1) {
          if(currentTemperature < 52) {    //startTemp - 2 should be 54 degree
            while(currentTemperature < 52){
              Serial.println("Temperature lower than melting point. Reheating...");
              currentTemperature = readTemp();  //Get temperature
              Serial.print("temperature is:  ");
              Serial.println(currentTemperature);
              myHeater->onestep(FORWARD, DOUBLE);
            }
        }
        applyWax();
//        Serial.println("cooling down");
//        currentTemperature = readTemp();  //Get temperature
//        if(currentTemperature >= 50) {  //set the current temperature to 1 degree lower than the melting point to prevent the wax from dripping
//          myHeater -> release();
//          while(currentTemperature > (startTemp - 2) - 1){
//              currentTemperature = readTemp();
//          }
//        }
        pointReached = 0;
      }
      delay(100);
      Serial.println("ACK\n");
    }
  }
}
