#ifndef PIDAPI_H_
#define PIDAPI_H_

#include "API.h"
#include "math.h"

typedef struct PID
{
  float kP;
  float kI;
  float kD;
  TaskHandle task;
  float (*get_real)();
  float (*get_target)();
  float error;
  float prev_error;
  float integral;
  float stable_delta;
}
#endif /* REXAPI_H_ */
