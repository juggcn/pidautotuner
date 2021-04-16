// PID automated tuning (Ziegler-Nichols/relay method) for Arduino and compatible boards
// Copyright (c) 2016-2020 jackw01
// This code is distrubuted under the MIT License, see LICENSE for details

#include "pidautotuner.h"

void vPIDAutotunerInit(PIDAutotuner_t *pxPIDAutotuner)
{
  pxPIDAutotuner->targetInputValue = 0;
  pxPIDAutotuner->loopInterval = 0;
  pxPIDAutotuner->znMode = ZNModeNoOvershoot;
  pxPIDAutotuner->cycles = 10;

  pxPIDAutotuner->outputValue = 0.0f;
  pxPIDAutotuner->outputValuePrev = 0.0f;

  pxPIDAutotuner->awMode = AWModeNormal;

  pxPIDAutotuner->kc = 1.0f;

  pxPIDAutotuner->integrator = 0.0f;
  pxPIDAutotuner->prevError = 0.0f;
}
// Set target input for tuning
void vPIDAutotunerSetTargetInputValue(PIDAutotuner_t *pxPIDAutotuner, float target)
{
  pxPIDAutotuner->targetInputValue = target;
}

// Set loop interval
void vPIDAutotunerSetLoopInterval(PIDAutotuner_t *pxPIDAutotuner, unsigned int interval)
{
  pxPIDAutotuner->loopInterval = interval;
}

// Set output range
void vPIDAutotunerSetOutputRange(PIDAutotuner_t *pxPIDAutotuner, float min, float max)
{
  pxPIDAutotuner->minOutput = min;
  pxPIDAutotuner->maxOutput = max;
}

// Set Ziegler-Nichols tuning mode
void vPIDAutotunerSetZNMode(PIDAutotuner_t *pxPIDAutotuner, ZNMode zn)
{
  pxPIDAutotuner->znMode = zn;
}

// Set tuning cycles
void vPIDAutotunerSetTuningCycles(PIDAutotuner_t *pxPIDAutotuner, unsigned int tuneCycles)
{
  pxPIDAutotuner->cycles = tuneCycles;
}

// Initialize all variables before loop
void vPIDAutotunerStartTuningLoop(PIDAutotuner_t *pxPIDAutotuner, long long us)
{
  pxPIDAutotuner->i = 0;      // Cycle counter
  pxPIDAutotuner->output = 1; // Current output state
  pxPIDAutotuner->outputValue = pxPIDAutotuner->maxOutput;
  pxPIDAutotuner->t1 = pxPIDAutotuner->t2 = us;                                    // Times used for calculating period
  pxPIDAutotuner->microseconds = pxPIDAutotuner->tHigh = pxPIDAutotuner->tLow = 0; // More time variables
  pxPIDAutotuner->max = -1000000;                                                  // Max input
  pxPIDAutotuner->min = 1000000;                                                   // Min input
  pxPIDAutotuner->pAverage = pxPIDAutotuner->iAverage = pxPIDAutotuner->dAverage = 0;
}

// Run one cycle of the loop
float fPIDAutotunerTunePID(PIDAutotuner_t *pxPIDAutotuner, float input, long long us)
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
  //long prevMicroseconds = microseconds;
  pxPIDAutotuner->microseconds = us;
  //float deltaT = microseconds - prevMicroseconds;

  // Calculate max and min
  pxPIDAutotuner->max = (pxPIDAutotuner->max > input) ? pxPIDAutotuner->max : input;
  pxPIDAutotuner->min = (pxPIDAutotuner->min < input) ? pxPIDAutotuner->min : input;

  // Output is on and input signal has risen to target
  if (pxPIDAutotuner->output && input > pxPIDAutotuner->targetInputValue)
  {
    // Turn output off, record current time as t1, calculate tHigh, and reset maximum
    pxPIDAutotuner->output = 0;
    pxPIDAutotuner->outputValue = pxPIDAutotuner->minOutput;
    pxPIDAutotuner->t1 = us;
    pxPIDAutotuner->tHigh = pxPIDAutotuner->t1 - pxPIDAutotuner->t2;
    pxPIDAutotuner->max = pxPIDAutotuner->targetInputValue;
  }

  // Output is off and input signal has dropped to target
  if (!pxPIDAutotuner->output && input < pxPIDAutotuner->targetInputValue)
  {
    // Turn output on, record current time as t2, calculate tLow
    pxPIDAutotuner->output = 1;
    pxPIDAutotuner->outputValue = pxPIDAutotuner->maxOutput;
    pxPIDAutotuner->t2 = us;
    pxPIDAutotuner->tLow = pxPIDAutotuner->t2 - pxPIDAutotuner->t1;

    // Calculate Ku (ultimate gain)
    // Formula given is Ku = 4d / πa
    // d is the amplitude of the output signal
    // a is the amplitude of the input signal
    float ku = (4.0 * ((pxPIDAutotuner->maxOutput - pxPIDAutotuner->minOutput) / 2.0)) /
               (M_PI * (pxPIDAutotuner->max - pxPIDAutotuner->min) / 2.0);

    // Calculate Tu (period of output oscillations)
    float tu = pxPIDAutotuner->tLow + pxPIDAutotuner->tHigh;

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
    if (pxPIDAutotuner->znMode == ZNModeBasicPID)
    {
      kpConstant = 0.6;
      tiConstant = 0.5;
      tdConstant = 0.125;
    }
    else if (pxPIDAutotuner->znMode == ZNModeLessOvershoot)
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
    pxPIDAutotuner->kp = kpConstant * ku;
    pxPIDAutotuner->ki = (pxPIDAutotuner->kp / (tiConstant * tu)) * pxPIDAutotuner->loopInterval;
    pxPIDAutotuner->kd = (tdConstant * pxPIDAutotuner->kp * tu) / pxPIDAutotuner->loopInterval;

    // Average all gains after the first two cycles
    if (pxPIDAutotuner->i > 1)
    {
      pxPIDAutotuner->pAverage += pxPIDAutotuner->kp;
      pxPIDAutotuner->iAverage += pxPIDAutotuner->ki;
      pxPIDAutotuner->dAverage += pxPIDAutotuner->kd;
    }

    // Reset minimum
    pxPIDAutotuner->min = pxPIDAutotuner->targetInputValue;

    // Increment cycle count
    pxPIDAutotuner->i++;
  }

  // If loop is done, disable output and calculate averages
  if (pxPIDAutotuner->i >= pxPIDAutotuner->cycles)
  {
    pxPIDAutotuner->output = 0;
    pxPIDAutotuner->outputValue = pxPIDAutotuner->minOutput;
    pxPIDAutotuner->kp = pxPIDAutotuner->pAverage / (pxPIDAutotuner->i - 1);
    pxPIDAutotuner->ki = pxPIDAutotuner->iAverage / (pxPIDAutotuner->i - 1);
    pxPIDAutotuner->kd = pxPIDAutotuner->dAverage / (pxPIDAutotuner->i - 1);
  }

  return pxPIDAutotuner->outputValue;
}

// Get PID constants after tuning
float fPIDAutotunerGetKp(PIDAutotuner_t *pxPIDAutotuner) { return pxPIDAutotuner->kp; };
float fPIDAutotunerGetKi(PIDAutotuner_t *pxPIDAutotuner) { return pxPIDAutotuner->ki; };
float fPIDAutotunerGetKd(PIDAutotuner_t *pxPIDAutotuner) { return pxPIDAutotuner->kd; };

// Is the tuning loop finished?
bool bPIDAutotunerIsFinished(PIDAutotuner_t *pxPIDAutotuner)
{
  return (pxPIDAutotuner->i >= pxPIDAutotuner->cycles);
}

void vPIDAutotunerSetAWMode(PIDAutotuner_t *pxPIDAutotuner, AWMode AW)
{
  pxPIDAutotuner->awMode = AW;
}

void vPIDAutotunerSetKc(PIDAutotuner_t *pxPIDAutotuner, float kc)
{
  pxPIDAutotuner->kc = kc;
}

void vPIDAutotunerSetSimpleTime(PIDAutotuner_t *pxPIDAutotuner, float t)
{
  pxPIDAutotuner->T = t;
}

float fPIDAutotunerTuneUpdate(PIDAutotuner_t *pxPIDAutotuner,
                              float setpoint, float measurement)
{
  if (pxPIDAutotuner->T > 0.0f)
  {
    float error = setpoint - measurement;

    float proportional = pxPIDAutotuner->kp * error;

    pxPIDAutotuner->integrator += pxPIDAutotuner->ki * pxPIDAutotuner->T * error;

    if (pxPIDAutotuner->awMode == AWModeAhead)
    {
      float limMinInt = 0.0f, limMaxInt = 0.0f;
      if (pxPIDAutotuner->maxOutput > proportional)
      {
        limMaxInt = pxPIDAutotuner->maxOutput - proportional;
      }
      if (pxPIDAutotuner->minOutput < proportional)
      {
        limMinInt = pxPIDAutotuner->minOutput - proportional;
      }

      if (pxPIDAutotuner->integrator > limMaxInt)
      {
        pxPIDAutotuner->integrator = limMaxInt;
      }
      else if (pxPIDAutotuner->integrator < limMinInt)
      {
        pxPIDAutotuner->integrator = limMinInt;
      }
    }

    pxPIDAutotuner->outputValuePrev = proportional +
                                      pxPIDAutotuner->integrator -
                                      pxPIDAutotuner->kd / pxPIDAutotuner->T * (error - pxPIDAutotuner->prevError);

    if (pxPIDAutotuner->outputValuePrev > pxPIDAutotuner->maxOutput)
    {
      pxPIDAutotuner->outputValue = pxPIDAutotuner->maxOutput;
    }
    else if (pxPIDAutotuner->outputValuePrev < pxPIDAutotuner->minOutput)
    {
      pxPIDAutotuner->outputValue = pxPIDAutotuner->minOutput;
    }
    else
    {
      pxPIDAutotuner->outputValue = pxPIDAutotuner->outputValuePrev;
    }

    if (pxPIDAutotuner->awMode == AWModeNormal)
    {
      pxPIDAutotuner->integrator += pxPIDAutotuner->kc * (pxPIDAutotuner->outputValue - pxPIDAutotuner->outputValuePrev);
    }
    //存储上一次的测量值
    pxPIDAutotuner->prevError = error;
  }

  return pxPIDAutotuner->outputValue;
}
