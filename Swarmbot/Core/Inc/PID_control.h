#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include "timer.h"

typedef struct
{
  float integral, proportional, derevative;
  float kp, ki, kd;

  float pre_error;
}PID;

uint8_t PID_init(PID *pid);

uint8_t reset_setpoint(PID* pid);

uint8_t tuning(PID *pid, float kp, float ki, float kd);

uint8_t PID_controller(PID *pid, Timer *timer, float setpoint, float input, float *output);

#endif
