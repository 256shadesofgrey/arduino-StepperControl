/*
StepperControl.cpp - A library for Arduino that allows simultaneous movement
of multiple stepper motors, and execution of an additional custom function
at the same time. This library requires the stepper motors to be controlled
by drivers with step, direction, and enable pins.

Theoretically it should work with up to 16 motors, but has only been tested
with 2, so I don't actually know if it will.

This library uses timer1, so it is likely to interfere with other things that
use it.

Version: 1.0
Last update (dd.mm.yyy): 09.04.2015
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

#include <Arduino.h>
#include "StepperControl.h"

//StepperControl StepperControl;

int StepperControl::motorsCount;
void (*StepperControl::isrA)();
int (*StepperControl::repeatInLoop)();
long *StepperControl::lowTime;
long *StepperControl::highTime;
int *StepperControl::stepStatus;
int *StepperControl::stepPin;
int *StepperControl::dirPin;
int *StepperControl::invertDir;
unsigned long *StepperControl::nextStateChange;
unsigned long *StepperControl::nextStep;
unsigned long StepperControl::interruptCounter = 0;
unsigned long StepperControl::stepsCounter = 0;
unsigned long StepperControl::stepsCounterCalc = 0;
unsigned long StepperControl::totalSteps = 0;
unsigned long StepperControl::idleCounter = 0;

uint8_t StepperControl::executionIndex = 0;
uint8_t StepperControl::calculationIndex = 1;
uint8_t StepperControl::waitForTask;
uint16_t StepperControl::taskList[3][bufferSize];
uint8_t *StepperControl::currentState;
uint8_t StepperControl::movementCompleted = 0;

/*Initialize class with user values.*/
StepperControl::StepperControl(int motorsCountE, int *enablePinE, int *invertEnableE, int *stepPinE, int *dirPinE, int *invertDirE, double *speedE, long *signalLengthE, long clockFrequencyE){
	clockFrequency = clockFrequencyE;
	motorsCount = motorsCountE;
	repeatInLoop = idleFunc;
	
	enablePin = (int*)calloc(motorsCount, sizeof(int));
	invertEnable = (int*)calloc(motorsCount, sizeof(int));
	stepPin = (int*)calloc(motorsCount, sizeof(int));
	dirPin = (int*)calloc(motorsCount, sizeof(int));
	invertDir = (int*)calloc(motorsCount, sizeof(int));
	speed = (double*)calloc(motorsCount, sizeof(double));
	signalLength = (long*)calloc(motorsCount, sizeof(long));
	
	//Time required for 1 full cycle, based on signalLength and speedE.
	movementTime = (long*)calloc(motorsCount, sizeof(long));
	
	lowTime = (long*)calloc(motorsCount, sizeof(long));
	highTime = (long*)calloc(motorsCount, sizeof(long));
	stepStatus = (int*)calloc(motorsCount, sizeof(int));
	nextStateChange = (unsigned long*)calloc(motorsCount, sizeof(unsigned long));
	currentState = (uint8_t*)calloc(motorsCount, sizeof(uint8_t));
	nextStep = (unsigned long*)calloc(motorsCount, sizeof(unsigned long));
	
	memcpy(enablePin, enablePinE, motorsCount*sizeof(int));
	memcpy(invertEnable, invertEnableE, motorsCount*sizeof(int));
	memcpy(stepPin, stepPinE, motorsCount*sizeof(int));
	memcpy(dirPin, dirPinE, motorsCount*sizeof(int));
	memcpy(invertDir, invertDirE, motorsCount*sizeof(int));
	memcpy(signalLength, signalLengthE, motorsCount*sizeof(long));
	
	setSpeed(speedE);
	timerSetup();
	
	for(int i = 0; i < motorsCount; i++){
		//currentState[i] = 1;
		stepStatus[i] = 0;
		pinMode(enablePin[i], OUTPUT);
		digitalWrite(enablePin[i], 1);
		pinMode(stepPin[i], OUTPUT);
		digitalWrite(stepPin[i], 0);
		pinMode(dirPin[i], OUTPUT);
		digitalWrite(dirPin[i], 0);
	}
	
	//Adding the function responsible for the movement to the interrupts.
	if(motorsCount > 0) StepperControl::attachInterrupt(interruptExec);
}

/*Configures timer 1 with the proper values.*/
void StepperControl::timerSetup(){
	cli(); //Stop interrupts

	//Set timer1 interrupts
	TCCR1A = 0; // Set entire TCCR1A register to 0
	TCCR1B = 0; // Same for TCCR1B
	TCNT1  = 0; // Initialize counter value to 0
	//Set compare match register to the given idle time.
	OCR1A = idleTime; //(must be <65536)
	//Turn on CTC mode
	TCCR1B |= (1 << WGM12);
	//Set CS10 and CS11 bits for 64 prescaler
	TCCR1B |= (1 << CS11) | (1 << CS10);
	//Enable timer compare interrupt
	//TIMSK1 |= (1 << OCIE1A);

	sei();//allow interrupts
}

/*Used internally to add the motor movement function to the ISR.
(Use addToLoop() if you want to execute a function with the movement.)*/
void StepperControl::attachInterrupt(void (*isr)()){
	isrA = isr; //Execute user function on interrupt.
}

/*Starting the interrupts for the movement.*/
void StepperControl::enableInterrupt(){
	cli();
	TCNT1  = 0; //Reset counter to 0.
	TIMSK1 |= (1 << OCIE1A); //Sets the timer compare A interrupt enable bit.
	sei();
}

/*Stopping the interrupts for the movement.*/
void StepperControl::disableInterrupt(){
	TIMSK1 &= ~_BV(OCIE1A); //Clears the timer compare interrupt enable bit. Timer continues to count without calling the isr.
}

/*Setting interrupt to TIMER1_COMPA_vect and adding a function to be executed.*/
ISR(TIMER1_COMPA_vect){
	cli();
	StepperControl::isrA();
	TCNT1 = 0;
	sei();
}

/*Adds a function to be executed in a loop together with the calculation of the movement.
The function has to return an integer, and if it returns 1, the movement stops. It should
also not return 2 (even though it would work in most cases), because moveAll() uses 2
to indicate an error.*/
void StepperControl::addToLoop(int (*inLoopFunc)()){
	if(inLoopFunc != NULL) repeatInLoop = inLoopFunc;
	else repeatInLoop = idleFunc;
}

/*Enables (1) or disables (0) the indicated motor.*/
void StepperControl::enableMotor(int enable, int motor){
	if(digitalRead(enablePin[motor]) != (enable+invertEnable[motor]) & 0x1){
		digitalWrite(enablePin[motor], (enable+invertEnable[motor]) & 0x1);
	}
}

/*Enables (1) or disables (0) all motors.*/
void StepperControl::enableAll(int enable){
	for(int i = 0; i < motorsCount; i++){
		enableMotor(enable, i);
	}
}

/*Sets movement speed of all motors, and adjusts all related variables.*/
void StepperControl::setSpeed(double *speedE){
	long speedStep;
	long sl;
	for(int i = 0; i < motorsCount; i++){
		if(speedE[i] >= 0) speed[i] = speedE[i];
		else speed[i] = 0;
		
		speedStep = (long)(1000000 / speed[i]);
		sl = signalLength[i]*2;
		if(speedStep > sl) movementTime[i] = speedStep;
		else movementTime[i] = sl;
	}
}

/*Returns the index of the highest value in the array.*/
int StepperControl::arrayMax(long *array, int arraySize){
	int maxIndex = 0;
	for(int i = 1; i < arraySize; i++){
		if(array[i] > array[maxIndex]){
			maxIndex = i;
		}
	}
	return maxIndex;
}

/*Returns the index of the smallest number of the array that is greater than floor. If there is no such number, returns -1.*/
int StepperControl::arrayMin(unsigned long *array, int arraySize, int floor){
	int minIndex = 0;
	
	//Finds first acceptable value.
	while(array[minIndex] <= (unsigned long)floor && minIndex < arraySize){
		minIndex++;
	}
	
	//If no value > 0 was found, return with error.
	if(minIndex >= arraySize) return -1;
	
	//Finding the lowest value that is > 0.
	for(int i = minIndex + 1; i < arraySize; i++){
		//Serial.println(array[i]);
		if(array[i] < array[minIndex] && array[i] > (unsigned long)floor){
			minIndex = i;
		}
	}
	
	return minIndex;
}

/*Performs a state change. Is called by performTask() for each motor
that has to be moved.*/
void StepperControl::move_t(int motor){
	//If all the required steps have completed, the completion flag is set and the function ends.
	if(stepsCounter > totalSteps){
		movementCompleted = 1;
		return;
	}
	++stepsCounter; //If this function was called, a state change actually took place, so the steps counter is increased.
	if(stepStatus[motor]){
		digitalWrite(stepPin[motor], HIGH);
		stepStatus[motor] = 0;
	}else{
		digitalWrite(stepPin[motor], LOW);
		stepStatus[motor] = 1;
	}
}

/*mode: 0 - overrun/both, 1 - underrun.
Checks if calculation and execution indices are far enough apart to continue
usual operation. Returns 1 if they are, 0 if not.*/
int StepperControl::bufferCheck(int mode){
	if(executionIndex%bufferSize == 0){
		if(mode == 1 && calculationIndex%bufferSize == 0) return 0;
		else if(mode == 0 && (calculationIndex%bufferSize == (bufferSize - 1) || calculationIndex%bufferSize == 0)) return 0;
		else return 1;
	}else{
		if(mode == 1 && calculationIndex%bufferSize == executionIndex%bufferSize) return 0;
		else if(mode == 0 && (calculationIndex%bufferSize == executionIndex%bufferSize || calculationIndex%bufferSize == (executionIndex-1)%bufferSize)) return 0;
		else return 1;
	}
}

/*Moves every motor that has to be moved.*/
void StepperControl::performTask(uint8_t taskIndex){
	//If there is a task to execute, do so.
	if(taskList[2][taskIndex] != 0){
		idleCounter = 0; //Resetting idle counter since at least one motor is being moved now.
		for(int i=0; i<motorsCount; i++){
			if(taskList[2][taskIndex] & (1 << i)) move_t(i);
		}
		taskList[2][taskIndex] = 0;
	}
}

/*This function is called on each interrupt. It reads the buffer, sets
the time for the next interrupt, and executes the action given in the
previous entry.*/
void StepperControl::interruptExec(){
	interruptCounter++; //Counting the interrupts. Currently serves no practical purpose (was used by older versions of this library).
	
	uint8_t exec = (uint8_t)(executionIndex%bufferSize); //Proper execution index based on buffer size.
	int bCheck = bufferCheck(1); //Buffer state.
	
	//As long as there is no waiting time given for the current task, but the buffer isn't empty, go to next task.
	while(taskList[1][exec] == 0 && bCheck){
		bCheck = bufferCheck(1);
		if(bCheck){
			++executionIndex;
		}else{
			--executionIndex;
		}
		exec = (uint8_t)(executionIndex%bufferSize);
	}
	
	//Adjusting values for new executionIndex.
	//exec = (uint8_t)(executionIndex%bufferSize);
	uint8_t exec1 = (uint8_t)(executionIndex-1);
	exec1 = exec1%bufferSize; //%bufferSize would be ignored it was added to the previous line. No idea, why.

	//If there are entries in the buffer, execute task from previous line (if there is anything to be done).
	if(bCheck){
		performTask(exec1);
	}
	
	//If the buffer underrun check failed, wait some time before trying again.
	if(!bCheck){
		OCR1A = idleTime;
		++idleCounter; //Increase idle counter. This will be used in future versions to subtract the idle time from the next step, allowing more consistent movement.
		return;
	}
	
	//If the time entries aren't empty, set new waiting time.
	//If the next task is further away than the timer would be able to count, wait for overflow and reduce overflow counter by 1.
	if(taskList[1][exec] > 2){
		--taskList[1][exec];
		OCR1A = 65535;
	}
	//If the next task is exactly as far as the timer would be able to count, set timer to max. value and move on to next entry.
	else if(taskList[1][exec] == 2 && taskList[0] == 0){
		taskList[1][exec] = taskList[1][exec]-2;
		OCR1A = 65535;
		++executionIndex;
	}
	//If the next task is due in less than the overflow time, set the timer to that time and move up to next entry.
	else if(taskList[1][exec] == 1 && taskList[0] > 0){
		OCR1A = taskList[0][exec];
		taskList[0][exec] = 0;
		--taskList[1][exec];
		++executionIndex;
	}
}

/*Sets the direction pin to the correct value based on the desired direction,
and whether the value is inverted.*/
void StepperControl::setDir(int motor, int dir){
	if(digitalRead(dirPin[motor]) != (dir+invertDir[motor]) & 0x1){
		digitalWrite(dirPin[motor], (dir+invertDir[motor]) & 0x1);
	}
}

/*Determines the amount of time before next state change. This function is called when the time
for the next step is below minimumTimerInterval.*/
void StepperControl::calculateNextStepTime(int motor, unsigned long *nextStep){
	//If the motor state is LOW, assigns highTime, and sets state to HIGH and vice versa.
	if(currentState[motor] == 0){
		nextStep[motor] += (unsigned long)highTime[motor];
		currentState[motor] = 1;
	}else{
		nextStep[motor] += (unsigned long)lowTime[motor];
		currentState[motor] = 0;
	}
}

/*Fills taskList until it's full. With current implementation it makes
sure that the list is really full when it stops the refilling, even if
some recently calculated values have been used already. This allows for
slightly longer independent execution of tasks in the interrupts, but
may cause an infinite loop, if the tasks are executed faster than new
tasks can be added. So make sure you have few enough motors that move
slowly enough.*/
void StepperControl::fillTaskList(uint16_t mask){
	uint16_t nextTimeAction = 0; //Each bit indicates a motor that will be moved after nextTaskTime.
	unsigned long nextTaskTime = 0; //Gives the amount of time before the next time something happens.
	int nextTaskTimeIndex = 0; //Index of the lowest value in nextStep that isn't 0.
	
	//Adds entries to the buffer as long as there is any space available.
	for( ; bufferCheck(); calculationIndex++){
		//Calculate the time until next step for each motor that is moved now.
		for(int i = 0; i < motorsCount; i++){
			if(nextStep[i] <= minimumTimerInterval && !(mask & (1 << i))) calculateNextStepTime(i, nextStep);
		}
		
		nextTaskTimeIndex = arrayMin(nextStep, motorsCount, minimumTimerInterval);
		if(nextTaskTimeIndex == -1) mask = 0xFFFF; //If there is no movement to be done, set mask to disable all motors.
		else nextTaskTime = nextStep[nextTaskTimeIndex];
		
		//Adding the time values to the buffer.
		if(mask != 0xFFFF){
			taskList[0][calculationIndex%bufferSize] = nextTaskTime%0x10000; //Remaining time after all overflows are done.
			taskList[1][calculationIndex%bufferSize] = (nextTaskTime/0x10000)+1; //Timer overflow count +1.
		}
		
		//Subtracts the previously determined lowest time before next step from the waiting time of each motor.
		//For each motor that reached minimumTimerInterval (has to be moved at this time), the nextTimeAction flag is set to 1.
		nextTimeAction = 0; //Resetting the action.
		for(int i = 0; i < motorsCount; i++){
			if(nextStep[i] > minimumTimerInterval) nextStep[i] -= nextTaskTime;
			if(nextStep[i] <= minimumTimerInterval && !(mask & (1 << i))){
				stepsCounterCalc++;
				nextTimeAction |= (1 << i);
			}
		}
		
		//Copying action flags to the buffer.
		taskList[2][calculationIndex%bufferSize] = nextTimeAction;
		
		//Debug output.
		/*Serial.print(calculationIndex);
		Serial.print(" ");
		Serial.print(taskList[0][calculationIndex%bufferSize]);
		Serial.print(" ");
		Serial.print(taskList[1][calculationIndex%bufferSize]);
		Serial.print(" ");
		Serial.println(taskList[2][calculationIndex%bufferSize]);*/
	}
}

/*Moves the motors simultaneously the number of steps given in the array.
The motors will start and stop at the same time, no matter the selected
speed and number of steps (the motor that would take the longest is taken
as reference, and the speed of other motors adjusted accordingly).*/
int StepperControl::moveAll(long *stepsE){
	movementCompleted = 0; //Resetting status.
	int exitStatus = 0; //Return value of the external function that can be added to the loop.
	uint16_t mask = 0; //Each bit contains info about what motor will be disabled (1 disabled, 0 enabled).
	
	long steps[motorsCount]; //Local array containing the number of steps to be executed by each motor.
	if(steps != NULL) memcpy(steps, stepsE, motorsCount*sizeof(long));
	else return 2; //End function if the given array is not valid.
	
	long minMovementDuration; //The lowest amount of time the slowest motor (or the one with the most steps to do) will need to complete it's movement.
	long durations[motorsCount]; //Used for calculation of time-dependent values. Has different contents.
	int slowestMotor = 0; //Index of the motor of which the movement will take the longest.
	int dir[motorsCount]; //Contains movement direction of the motors.
	
	totalSteps = 0; //Resetting the number of steps to be done.
	
	int slMin; //Index of the motor with the lowest signalLength value among the active ones.
	unsigned long *slp; //Points to a copy of signalLength.
	slp = (unsigned long*)calloc(motorsCount, sizeof(unsigned long));
	memcpy(slp, signalLength, motorsCount*sizeof(unsigned long)); //Sloppy (signalLength has type long*), but if signalLength contains negative values, there is a bigger problem.
	
	//The direction pins of all motors are set here depending on whether or not the number of steps is negative.
	//The total number of state changes to be done is also calculated here.
	for(int i = 0; i < motorsCount; i++){
		if(steps[i] < 0){
			dir[i] = 1;
			steps[i] = -steps[i];
		}else{
			dir[i] = 0;
		}
		totalSteps += steps[i];
		setDir(i, dir[i]); //Setting the correct values to the direction pins.
		
		durations[i] = movementTime[i] * steps[i];
		
		if(!steps[i]){
			mask |= (1 << i);
			slp[i] = 0; //Causes arrayMin in the next step to ignore the motors that will be inactive.
		}
	}
	totalSteps = totalSteps*2;
	
	//Set starting conditions (without this part the signal polarity can be wrong).
	slMin = arrayMin(slp, motorsCount, 0);
	for(int i = 0; i < motorsCount; i++){
		if(signalLength[i] > signalLength[slMin] && !(mask & (1 << i))){
			stepStatus[i] = 1;
			currentState[i] = 1;
		}else{
			stepStatus[i] = 0;
			currentState[i] = 0;
		}
	}
	free(slp);//We won't need slp any more.

	//Here, the form and duration of the impulses is determined.
	slowestMotor = arrayMax(durations, motorsCount);
	minMovementDuration = durations[slowestMotor];
	for(int i = 0; i < motorsCount; i++){
		durations[i] = minMovementDuration / steps[i];
		if(durations[i] > signalLength[i]) durations[i] -= signalLength[i];
		else durations[i] = 0;
		
		if(durations[i] > 0){
			lowTime[i] = (long)((clockFrequency/(64*(1000000.0/durations[i])))/*-1*/);
			highTime[i] = (long)((clockFrequency/(64*(1000000.0/signalLength[i])))/*-1*/);
		}else{
			lowTime[i] = 0;
			highTime[i] = 0;
		}
	}
	
	//Actual movement starts to happen here.
	//Setting all motors to HIGH at start.
	taskList[3][0] = 0xFFFF;
	fillTaskList(mask);//Preparing the buffer.
	enableInterrupt();//Starting movement.
	while(!movementCompleted){
		exitStatus = repeatInLoop(); //External function to be executed during the movement.
		if(exitStatus == 1) break; //If the external function returns 1, the movemen stops.
		fillTaskList(mask);
	}
	disableInterrupt();//Stopping interrupts, so that they won't interfere with the program.
	
	//Movement ended, resetting values.
	stepsCounter = 0;
	stepsCounterCalc = 0;
	movementCompleted = 0;
	executionIndex = 0;
	calculationIndex = 1;
	clearTaskList();
	for(int i = 0; i < motorsCount; i++){
		currentState[i] = 0;
		stepStatus[i] = 0;
		nextStep[i] = 0;
	}
	
	return exitStatus;
}

//Dummy function that is executed if addToLoop() wasn't called by the user.
int StepperControl::idleFunc(){
	return 0;
}

//Sets all values of a given array to 0.
void StepperControl::initArray(unsigned long *array, int arraySize){
	for(int i = 0; i < arraySize; i++){
		array[i] = 0;
	}
}

//Removes all entries from the buffer.
void StepperControl::clearTaskList(){
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < bufferSize; j++){
			taskList[i][j] = 0;
		}
	}
}