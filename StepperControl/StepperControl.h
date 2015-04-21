/*
StepperControl.h - A library for Arduino that allows simultaneous movement
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

/*
Changelog:
v1.2 (21.04.2015)
- Added enabledMotors[] to check what motors are on.
- Fixed a bug that caused the motor movement to stop if the execution caught up with the calculation.
    This bug appeared when the buffer size was too small, or the user function took too much time to execute.
- Set bufferSize to 64 (384 bytes).

v1.1 (14.04.2015)
- Added state change/step counter for each individual motor (individualStepsCounter[<motor>]/2 = completed steps of <motor>). Use resetIndividualStepsCounter() to reset it.
- Added toggleTimer(<0|1>) function. Now it is possible to pause the movement, rather than just abort it.
- Clean-up:
	- These variables are now protected: movementCompleted, *nextStep, *nextStateChange, stepsCounterCalc, motorsCount,
	    *lowTime, *highTime, *stepStatus, *stepPin, *dirPin, *invertDir, idleCounter, stepsCounter, totalSteps, 
		executionIndex, calculationIndex, taskList[][], *currentState
	- These functions are now protected: clearTaskList(), setDir(), interruptExec(), performTask(), move_t(), (*repeatInLoop)()
	- Removed unused variables: waitForTask
	- Removed most of the unused commented-out code

v1.0 (09.04.2015)
Initial release.
*/

#ifndef _StepperControl
#define _StepperControl
#include <Arduino.h>

//Has to be: 256%bufferSize = 0. Other values might work, but may occasionally cause the buffer to be used only partially.
//The buffer uses 3 uint16_t per entry, so the memory usage is 6*bufferSize bytes (96 bytes for 16 entries).
#define bufferSize 64
//250 is 1 ms with a prescaler of 64 and a clock speed of 16MHz.
//Adjust this value if you want to wait a different amount of time when there are no entries in the buffer.
#define idleTime 250
//The minimum number of timer ticks per interrupt (25 = 0.1 ms). Try adding this time to the signal length, if your motors
//move irregularly.
//Increasing this value will reduce overall precision of the interrupt timing, but too low values may lead to the timer
//being beyond the given value by the time the ISR is finished, and the interrupts re-enabled, resulting in a waiting time
//of up to 65536 ticks (a full cycle of the counter).
#define minimumTimerInterval 25

class StepperControl{
	public:
		//Final movement speed will be determined by *speedE or *signalLengthE, whichever indicates a slower movement. All values in *speedE can be set to 0 for max. speed.
		StepperControl(int motorsCountE, int *enablePinE, int *invertEnableE, int *stepPinE, int *dirPinE, int *invertDirE, double *speedE, long *signalLengthE, long clockFrequencyE = 16000000);
		
		//---User functions---
		int moveAll(long *steps);
		void enableMotor(int enable = 0, int motor = 0);
		void enableAll(int enable = 0);
		void setSpeed(double *speedE);
		void timerSetup();
		void resetIndividualStepsCounter();
		void toggleTimer(int on = 1);
		static void addToLoop(int (*inLoopFunc)());
		
		//---Functions intended for internal use, but could be useful for external applications.---
		static int idleFunc();
		
		//---Information variables for the user.---
		static unsigned long *individualStepsCounter;
		static uint8_t *enabledMotors;
		static unsigned long interruptCounter;
		
		//---Functions for internal use that have to be public for technical reasons.---
		static void (*isrA)();
		
		
	protected:
		int *enablePin;
		int *invertEnable;
		double *speed;
		long *movementTime;
		long clockFrequency;
		long *signalLength; //time required to complete 1 step.
		static unsigned long stepsCounterCalc;
		static unsigned long *nextStateChange;
		static unsigned long *nextStep;
		static uint8_t movementCompleted;
		static int motorsCount;
		static long *lowTime;
		static long *highTime;
		static int *stepStatus;
		static int *stepPin;
		static int *dirPin;
		static int *invertDir;
		static unsigned long idleCounter;
		static unsigned long stepsCounter;
		static unsigned long totalSteps;
		static uint8_t executionIndex; //The actual index will be calculated with %bufferSize.
		static uint8_t calculationIndex; //The actual index will be calculated with %bufferSize.
		static uint16_t taskList[3][bufferSize]; //Contains the time difference between tasks, number of overflows before execution, and an indicator of which task to execute.
		static uint8_t *currentState;
		
		static int (*repeatInLoop)();
		static int bufferCheck(int mode = 0);
		static void clearTaskList();
		static void interruptExec();
		static void performTask(uint8_t taskIndex);
		static void move_t(int motor);
		
		void setDir(int motor = 0, int dir = 0);
		void attachInterrupt(void (*isr)());
		void disableInterrupt();
		void enableInterrupt();
		int arrayMax(long *array, int arraySize);
		int arrayMin(unsigned long *array, int arraySize, int floor = minimumTimerInterval);
		void initArray(unsigned long *array, int arraySize);
		void calculateNextStepTime(int motor, unsigned long *nextStep);
		void fillTaskList(uint16_t mask);
};


#endif