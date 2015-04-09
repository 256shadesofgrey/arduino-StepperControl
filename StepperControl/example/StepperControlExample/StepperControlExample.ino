/*
StepperControlExample.ino - Example application for the StepperControl library.
StepperControl - A library for Arduino that allows simultaneous movement
of multiple stepper motors, and execution of an additional custom function
at the same time. This library requires the stepper motors to be controlled
by drivers with step, direction, and enable pins.

Theoretically it should work with up to 16 motors, but has only been tested
with 2, so I don't actually know if it will.

This library uses timer1, so it is likely to interfere with other things that
use it.
*/

/*
  Copyright 2015 Dmitri Ranfft
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <StepperControl.h>

#define mc 2
//How many motors/drivers you use.
int motorsCountE = mc;
//Enable pins for each motor/driver.
int enablePinE[mc] = {A5, A5};
//Whether any of the enable pins will be indicating the inverse of your intended movement.
//This might be helpful if you need to adjust your software quicly to a new hardware setup.
int invertEnableE[mc] = {1, 1};
//Step pins of the motors/drivers.
int stepPinE[mc] = {A3, A0};
//Direction pins.
int dirPinE[mc] = {A4, A1};
//Same as invertEnableE, but for direction.
int invertDirE[mc] = {1, 0};
//Movement speed of the motors in steps per second.
//This speed will be ignored if signalLength requires more time for the movement.
double speedE[mc] = {500.0, 500.0};
//Minimum amount of time the step pins are required to spend in one state (HIGH or LOW).
//These values are always used for the HIGH time, and the LOW time can be increased
//by speedE. To determine this value, read the datasheet of the motors you are using.
long signalLengthE[mc] = {1500, 1000};

StepperControl motors = StepperControl(motorsCountE, enablePinE, invertEnableE, stepPinE, dirPinE, invertDirE, speedE, signalLengthE);

//How many steps each motor has to do.
//Negative values will make the motor move in the reverse direction.
//0 will make the motor stand still while the other ones move.
long steps[mc] = {110, 230};

//Let's see how much stuff you can do while the motors move.
int externalFunctionCalls = 0;

//Any custom function you want to execute while the motors are moving.
//Must be fast enough to finish before the buffer runs out of entries.
//Buffer size is 16 in v1.0. This value can currently (v1.0) only be
//adjusted by setting the bufferSize value in the header file.
int externalFunction(){
  externalFunctionCalls++;
  Serial.println(externalFunctionCalls);
  
  //If the external function returns 1, the movement is interrupted.
  //Note: moveAll() returns the same value as the custom function,
  //except if it received a NULL pointer as parameter, in which case
  //it returns 2. So be careful with the 2's that you get from
  //moveAll().
  if(externalFunctionCalls >= 10000){
    Serial.println("Premature interrupt. Reseting counter.");
    //Because this function is called before the loop breaks, it will
    //continue to be called, and hence the number would continue increasing
    //once for each call to moveAll(). Try it by deleting or commenting out
    //the next line.
    externalFunctionCalls = 0;
    //Some time to read the message.
    delay(3000);
    return 1;
  }else{
    return 0;
  }
}

void setup() {
  Serial.begin(115200);
  //timerSetup() has to be called within setup(), because the setup() function seems to reset the timer configuration at start.
  motors.timerSetup();
  //Here we add the custom function to the loop in which the calculation of the movement happens.
  motors.addToLoop(externalFunction);
}

void loop() {
  motors.enableAll(1); //Making sure all motors are on.
  motors.moveAll(steps); //Move all motors (steps is an array with entries of the type long int).
  motors.enableAll(0); //Turning all motors off.
  steps[1] = -steps[1]; //To move in the opposite direction, give a negative number of steps.
  delay(500);
}

