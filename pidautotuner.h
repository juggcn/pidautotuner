// PID automated tuning (Ziegler-Nichols/relay method) for Arduino and compatible boards
// Copyright (c) 2016-2020 jackw01
// This code is distrubuted under the MIT License, see LICENSE for details

#ifndef PIDAUTOTUNER_H
#define PIDAUTOTUNER_H

#include "stdbool.h"

//#pragma anon_unions
#pragma pack(1)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Constants for Ziegler-Nichols tuning mode
typedef enum
{
  ZNModeBasicPID,
  ZNModeLessOvershoot,
  ZNModeNoOvershoot
} ZNMode;

typedef struct
{
  float targetInputValue;
  unsigned int loopInterval;
  float minOutput, maxOutput;
  ZNMode znMode;
  unsigned int cycles;

  unsigned int i;
  bool output;
  float outputValue;
  long long microseconds, t1, t2, tHigh, tLow;
  float max, min;
  float pAverage, iAverage, dAverage;
  float kp, ki, kd;

  /* Sample time (in seconds) */
  float T;

  /* Controller "memory" */
  float integrator;
  float prevMeasurement; /* Required for differentiator */

} PIDAutotuner_t;

// Configure parameters for PID tuning
// See README for more details - https://github.com/jackw01/arduino-pid-autotuner/blob/master/README.md
// targetInputValue: the target value to tune to
// loopInterval: PID loop interval in microseconds - must match whatever the PID loop being tuned runs at
// outputRange: min and max values of the output that can be used to control the system (0, 255 for analogWrite)
// znMode: Ziegler-Nichols tuning mode (znModeBasicPID, znModeLessOvershoot, znModeNoOvershoot)
// tuningCycles: number of cycles that the tuning runs for (optional, default is 10)
extern void vPIDAutotunerInit(PIDAutotuner_t *pxPIDAutotuner);
extern void vPIDAutotunerSetTargetInputValue(PIDAutotuner_t *pxPIDAutotuner, float target);
extern void vPIDAutotunerSetLoopInterval(PIDAutotuner_t *pxPIDAutotuner, unsigned int interval);
extern void vPIDAutotunerSetOutputRange(PIDAutotuner_t *pxPIDAutotuner, float min, float max);
extern void vPIDAutotunerSetZNMode(PIDAutotuner_t *pxPIDAutotuner, ZNMode zn);
extern void vPIDAutotunerSetTuningCycles(PIDAutotuner_t *pxPIDAutotuner, unsigned int tuneCycles);

// Must be called immediately before the tuning loop starts
extern void vPIDAutotunerStartTuningLoop(PIDAutotuner_t *pxPIDAutotuner, long long us);

// Automatically tune PID
// This function must be run in a loop at the same speed as the PID loop being tuned
// See README for more details - https://github.com/jackw01/arduino-pid-autotuner/blob/master/README.md
extern float fPIDAutotunerTunePID(PIDAutotuner_t *pxPIDAutotuner, float input, long long us);

// Get results of most recent tuning
extern float fPIDAutotunerGetKp(PIDAutotuner_t *pxPIDAutotuner);
extern float fPIDAutotunerGetKi(PIDAutotuner_t *pxPIDAutotuner);
extern float fPIDAutotunerGetKd(PIDAutotuner_t *pxPIDAutotuner);

extern bool bPIDAutotunerIsFinished(PIDAutotuner_t *pxPIDAutotuner); // Is the tuning finished?

extern float fPIDAutotunerTuneUpdate(PIDAutotuner_t *pxPIDAutotuner,
                                     float setpoint, float measurement);

#pragma pack()

#endif
