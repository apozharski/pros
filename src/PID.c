#include "math.h"
#include "PID.h"

PID* init_PID(float (*real)(),float (*target)())
{
  PID *ret = malloc(sizeof(PID)); 
  PID n = {.get_real = real, .get_target = target};
  &ret = n;
  return ret;
}

void start_PID(PID *pid)
{
}

float get_PID_output(PID *pid)
{
}

void set_kp(PID *pid, float P)
{
  pid->kP = P;
}

void set_ki(PID *pid, float I)
{
  pid->kI = I;
}

void set_kd(PID *pid, float D)
{
  pid->kD = D;
}

void set_delta(PID *pid, float delta)
{
  pid->stable_delta = delta;
}

float get_error(PID *pid)
{
  return pid->error;
}

float get_integral(PID *pid)
{
  return pid->integral;
}

float get_deriv(PID *pid)
{
  return (pid->error-pid->prev_error);
}

bool is_stable(PID *pid)
{
  return (abs(pid->error-pid)<pid->stable_delta);
}
