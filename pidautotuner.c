// PID automated tuning (Ziegler-Nichols/relay method) for Arduino and compatible boards
// Copyright (c) 2016-2020 jackw01
// This code is distrubuted under the MIT License, see LICENSE for details

#include "pidautotuner.h"

void vPIDAutotunerInit(PIDAutotuner_t *pxPID)
{
  pxPID->targetInputValue = 0;
  pxPID->loopInterval = 0;
  pxPID->znMode = ZNModeBasicPID;
  pxPID->cycles = 10;

  pxPID->outputValue = 0.0f;

  pxPID->integrator = 0.0f;
  pxPID->prevError = 0.0f;
  pxPID->differentiator = 0.0f;
  pxPID->prevMeasurement = 0.0f;
}
// Set target input for tuning
void vPIDAutotunerSetTargetInputValue(PIDAutotuner_t *pxPID, float target)
{
  pxPID->targetInputValue = target;
}

// Set loop interval
void vPIDAutotunerSetLoopInterval(PIDAutotuner_t *pxPID, int interval)
{
  pxPID->loopInterval = interval;
}

// Set output range
void vPIDAutotunerSetOutputRange(PIDAutotuner_t *pxPID, float min, float max)
{
  pxPID->minOutput = min;
  pxPID->maxOutput = max;

  pxPID->minOutputInt = min;
  pxPID->maxOutputInt = max;
}

// Set Ziegler-Nichols tuning mode
void vPIDAutotunerSetZNMode(PIDAutotuner_t *pxPID, ZNMode zn)
{
  pxPID->znMode = zn;
}

// Set tuning cycles
void vPIDAutotunerSetTuningCycles(PIDAutotuner_t *pxPID, int tuneCycles)
{
  pxPID->cycles = tuneCycles;
}

// Initialize all variables before loop
void vPIDAutotunerStartTuningLoop(PIDAutotuner_t *pxPID, unsigned int us)
{
  pxPID->i = 0;      // Cycle counter
  pxPID->output = 1; // Current output state
  pxPID->outputValue = pxPID->maxOutput;
  pxPID->t1 = pxPID->t2 = us;                           // Times used for calculating period
  pxPID->microseconds = pxPID->tHigh = pxPID->tLow = 0; // More time variables
  pxPID->max = -100000000;                              // Max input
  pxPID->min = 100000000;                               // Min input
  pxPID->pAverage = pxPID->iAverage = pxPID->dAverage = 0;
}

// Run one cycle of the loop
float fPIDAutotunerTunePID(PIDAutotuner_t *pxPID, float input, unsigned int us)
{
  // Useful information on the algorithm used (Ziegler-Nichols method/Relay method)
  // http://www.processcontrolstuff.net/wp-content/uploads/2015/02/relay_autot-2.pdf
  // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
  // https://www.cds.caltech.edu/~murray/courses/cds101/fa04/caltech/am04_ch8-3nov04.pdf

  // Basic explanation of how this works:
  //  * Turn on the output of the PID controller to full power
  //  * Wait for the output of the system being tuned to reach the target input value
  //      and then turn the controller output off
  //  * Wait for the output of the system being tuned to decrease below the target input
  //      value and turn the controller output back on
  //  * Do this a lot
  //  * Calculate the ultimate gain using the amplitude of the controller output and
  //      system output
  //  * Use this and the period of oscillation to calculate PID gains using the
  //      Ziegler-Nichols method

  // Calculate time delta
  // long prevMicroseconds = microseconds;
  pxPID->microseconds = us;
  // float deltaT = microseconds - prevMicroseconds;

  // Calculate max and min
  pxPID->max = (pxPID->max > input) ? pxPID->max : input;
  pxPID->min = (pxPID->min < input) ? pxPID->min : input;

  // Output is on and input signal has risen to target
  if (pxPID->output && input > pxPID->targetInputValue)
  {
    // Turn output off, record current time as t1, calculate tHigh, and reset maximum
    pxPID->output = 0;
    pxPID->outputValue = pxPID->minOutput;
    pxPID->t1 = us;
    pxPID->tHigh = pxPID->t1 - pxPID->t2;
    pxPID->max = pxPID->targetInputValue;
  }

  // Output is off and input signal has dropped to target
  if (!pxPID->output && input < pxPID->targetInputValue)
  {
    // Turn output on, record current time as t2, calculate tLow
    pxPID->output = 1;
    pxPID->outputValue = pxPID->maxOutput;
    pxPID->t2 = us;
    pxPID->tLow = pxPID->t2 - pxPID->t1;

    // Calculate Ku (ultimate gain)
    // Formula given is Ku = 4d / πa
    // d is the amplitude of the output signal
    // a is the amplitude of the input signal
    float ku = (4.0 * ((pxPID->maxOutput - pxPID->minOutput) / 2.0)) /
               (M_PI * (pxPID->max - pxPID->min) / 2.0);

    // Calculate Tu (period of output oscillations)
    float tu = pxPID->tLow + pxPID->tHigh;

    // How gains are calculated
    // PID control algorithm needs Kp, Ki, and Kd
    // Ziegler-Nichols tuning method gives Kp, Ti, and Td
    //
    // Kp = 0.6Ku = Kc
    // Ti = 0.5Tu = Kc/Ki
    // Td = 0.125Tu = Kd/Kc
    //
    // Solving these equations for Kp, Ki, and Kd gives this:
    //
    // Kp = 0.6Ku
    // Ki = Kp / (0.5Tu)
    // Kd = 0.125 * Kp * Tu

    // Constants
    // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method

    float kpConstant, tiConstant, tdConstant;
    if (pxPID->znMode == ZNModeBasicPID)
    {
      kpConstant = 0.6;
      tiConstant = 0.5;
      tdConstant = 0.125;
    }
    else if (pxPID->znMode == ZNModeLessOvershoot)
    {
      kpConstant = 0.33;
      tiConstant = 0.5;
      tdConstant = 0.33;
    }
    else
    { // Default to No Overshoot mode as it is the safest
      kpConstant = 0.2;
      tiConstant = 0.5;
      tdConstant = 0.33;
    }

    // Calculate gains
    pxPID->kp = kpConstant * ku;
    pxPID->ki = (pxPID->kp / (tiConstant * tu)) * pxPID->loopInterval;
    pxPID->kd = (tdConstant * pxPID->kp * tu) / pxPID->loopInterval;

    // Average all gains after the first two cycles
    if (pxPID->i > 1)
    {
      pxPID->pAverage += pxPID->kp;
      pxPID->iAverage += pxPID->ki;
      pxPID->dAverage += pxPID->kd;
    }

    // Reset minimum
    pxPID->min = pxPID->targetInputValue;

    // Increment cycle count
    pxPID->i++;
  }

  // If loop is done, disable output and calculate averages
  if (pxPID->i >= pxPID->cycles)
  {
    pxPID->output = 0;
    pxPID->outputValue = pxPID->minOutput;
    pxPID->kp = pxPID->pAverage / (pxPID->i - 1);
    pxPID->ki = pxPID->iAverage / (pxPID->i - 1);
    pxPID->kd = pxPID->dAverage / (pxPID->i - 1);
  }

  return pxPID->outputValue;
}

// Get PID constants after tuning
float fPIDAutotunerGetKp(PIDAutotuner_t *pxPID) { return pxPID->kp; };
float fPIDAutotunerGetKi(PIDAutotuner_t *pxPID) { return pxPID->ki; };
float fPIDAutotunerGetKd(PIDAutotuner_t *pxPID) { return pxPID->kd; };

// Is the tuning loop finished?
bool bPIDAutotunerIsFinished(PIDAutotuner_t *pxPID)
{
  return (pxPID->i >= pxPID->cycles);
}

float fPIDAutotunerTuneUpdate(PIDAutotuner_t *pxPID,
                              float setpoint, float measurement)
{
  /*
   * Error signal
   */
  float error = setpoint - measurement;

  /*
   * Proportional
   */
  float proportional = pxPID->kp * error;

  /*
   * Integral
   */
  pxPID->integrator = pxPID->integrator + pxPID->ki * pxPID->T * (error + pxPID->prevError);

  /* Anti-wind-up via integrator clamping */

#if 0

  if (pxPID->integrator > pxPID->maxOutputInt)
  {

    pxPID->integrator = pxPID->maxOutputInt;
  }
  else if (pxPID->integrator < pxPID->minOutputInt)
  {

    pxPID->integrator = pxPID->minOutputInt;
  }

#else

  float minOutputInt = pxPID->minOutput, maxOutputInt = pxPID->maxOutput;

  if (pxPID->maxOutput > proportional)
  {
    maxOutputInt = pxPID->maxOutput - proportional;
  }
  if (pxPID->minOutput < proportional)
  {
    minOutputInt = pxPID->minOutput - proportional;
  }

  if (pxPID->integrator > maxOutputInt)
  {
    pxPID->integrator = maxOutputInt;
  }
  else if (pxPID->integrator < minOutputInt)
  {
    pxPID->integrator = minOutputInt;
  }

#endif

  /*
   * Derivative (band-limited differentiator)
   */

  pxPID->differentiator = -(2.0f * pxPID->kd * (measurement - pxPID->prevMeasurement) /* Note: derivative on measurement, therefore minus sign in front of equation! */
                            + (2.0f * pxPID->tau - pxPID->T) * pxPID->differentiator) /
                          (2.0f * pxPID->tau + pxPID->T);

  /*
   * Compute output and apply limits
   */
  pxPID->out = proportional + pxPID->integrator + pxPID->differentiator;

  if (pxPID->out > pxPID->maxOutput)
  {

    pxPID->out = pxPID->maxOutput;
  }
  else if (pxPID->out < pxPID->minOutput)
  {

    pxPID->out = pxPID->minOutput;
  }

  /* Store error and measurement for later use */
  pxPID->prevError = error;
  pxPID->prevMeasurement = measurement;

  pxPID->outputValue = pxPID->out;

  /* Return controller output */
  return pxPID->out;
}
