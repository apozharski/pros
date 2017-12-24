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
  float output;
} PID;

PID init_PID(float (*get_real)(),float (*get_target)());

void start_PID(PID *pid);

float get_PID_output(PID *pid);

void set_kp(PID *pid, float P);

void set_ki(PID *pid, float I);

void set_kd(PID *pid, float D);

void set_delta(PID *pid, float delta);

float get_error(PID *pid);

float get_integral(PID *pid);

float get_deriv(PID *pid);

bool is_stable(PID *pid);

#endif /* REXAPI_H_ */
