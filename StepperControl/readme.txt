Documentation for the Arduino StepperControl library by Dmitri Ranfft.

Last update (dd.mm.yyyy): 21.04.2015
For version: 1.2
--------------------------------------------------------------------------------------------------------------------------------
Description of the library

This is a library for Arduino that allows simultaneous movement of up to 16 stepper motors,
and an execution of an additional task without interferance with the motor movement.

This library uses interrupts on timer1 for the movement. Outside of the interrupts
a buffer is filled with timer values for the following interrupts and information about what
pins need a state change to make the motors move. It also executes an additional function,
that the user can attach to the loop that does the calculation.

The ISR reads entries from the buffer and performs the given tasks/motor movements.
Because there is a buffer, the calculation can be suspended for a while without causing
interference with the motor movement.
--------------------------------------------------------------------------------------------------------------------------------
Changelog:
v1.2 (21.04.2015)
- Added enabledMotors[] to check what motors are on.
- Fixed a bug that caused the motor movement to stop if the execution caught up with the calculation.
    This bug appeared when the buffer size was too small, or the user function took too much time to execute.
- Set bufferSize to 64 (384 bytes).

v1.1
- Added state change/step counter for each individual motor (individualStepsCounter[<motor>]/2 = completed steps of <motor>).
  Use resetIndividualStepsCounter() to reset it.
- Added toggleTimer(<0|1>) function. Now it is possible to pause the movement, rather than just abort it.
- Clean-up:
    - These variables are now protected: movementCompleted, *nextStep, *nextStateChange, stepsCounterCalc, motorsCount,
      *lowTime, *highTime, *stepStatus, *stepPin, *dirPin, *invertDir, idleCounter, stepsCounter, totalSteps, 
      executionIndex, calculationIndex, taskList[][], *currentState
    - These functions are now protected: clearTaskList(), setDir(), interruptExec(), performTask(), move_t(), (*repeatInLoop)()
    - Removed unused variables: waitForTask
    - Removed most of the unused commented-out code

v1.0
Initial release.
--------------------------------------------------------------------------------------------------------------------------------
Usage

See example.
--------------------------------------------------------------------------------------------------------------------------------
Solutions to possible problems

1.
Problem: The motor movement is interrupted when the user function is called.

Solution: This can happen if the user function takes a long time to execute.
Try increasing the buffer size (the variable bufferSize in the header file).
Keep in mind that only specific values will work, and that increasing the buffer
size will significantly increase RAM usage.

2.
Problem: The step pins give irregular signals.

Solution: If the variation is less than 0.1 ms (25 ticks), it works as intended. But if it is causing you problems,
try reducing the minimum timer interval (minimumTimerInterval in the header file). If the variation is
bigger than that, try increasing it instead. Keep in mind that if minimumTimerInterval is too small,
the timer might jump over the target value, causing it to wait for 2^16 steps. And a too big value will
increase irregularities in the timing.
--------------------------------------------------------------------------------------------------------------------------------
Possible future features

If I ever get around to working on this library again, I intend to add the following things:
- dynamic adjustment of the buffer size, idle time and minimum timer interval
- add an option to subtract the idle time from the next task, to give more time to the custom function
- add an option to allow proportional signals (rather than static length of the HIGH state, and variable length of the LOW state)
- splitting the library in two (one responsible for the motor, and the other responsible for the timer+buffer)
--------------------------------------------------------------------------------------------------------------------------------