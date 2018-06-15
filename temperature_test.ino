/***************************************************
 This example reads HighTemperature Sensor.
 
 Created 2016-1-13
 By berinie Chen <bernie.chen@dfrobot.com>
 
 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/

/***********Notice and Trouble shooting***************
 1.Connection and Diagram can be found here  https://www.dfrobot.com/wiki/index.php?title=HighTemperatureSensor_SKU:SEN0198
 2.This code is tested on Arduino Uno.
 ****************************************************/
#include<DFRobotHighTemperatureSensor.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

const float voltageRef = 5.000; //Set reference voltage,you need test your IOREF voltage. 
//const float voltageRef = 3.300; 
int HighTemperaturePin = A0;  //Setting pin
DFRobotHighTemperature PT100 = DFRobotHighTemperature(voltageRef); //Define an PT100 object



//
////------------------for stepper motor---------------------//
//
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
 Adafruit_MotorShield AFMS_level2 = Adafruit_MotorShield(0x60); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4), port #1 is M1 and M2
Adafruit_StepperMotor *myHeater = AFMS_level2.getStepper(200, 1); //for the heating resistor
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 2);
////------------------for stepper motor---------------------//



void setup(void) {
  Serial.begin(9600);
  AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  AFMS_level2.begin(1000);
  
  myMotor->setSpeed(100);  // 50 rpm 
  myHeater->setSpeed(50);  
}

void loop(void) {
  int temperature = PT100.readTemperature(HighTemperaturePin);  //Get temperature
  Serial.print("temperature:  ");
  Serial.print(temperature);
  Serial.println("  ^C");
  if(temperature >= 45){
    myHeater->setSpeed(0);
    Serial.println("Overheat");
  }
  else{
    myHeater->setSpeed(50);
    myHeater->step(100, FORWARD, MICROSTEP);  //heat controlled by PWM
  }
  
  myMotor->step(100, FORWARD, MICROSTEP);
  //myMotor->step(100, BACKWARD, SINGLE); 
  delay(10); //just here to slow down the output so it is easier to read
  
}

