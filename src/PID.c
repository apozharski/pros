#include "math.h"
#include "PID.h"
#include "task.h"
#include "API.h"

#define MOTOR_REFRESH_TIME 20

void pid_func(void *param)
{
  PID *pid = (PID*) param;

  unsigned long loopTime;
  
  while(true)
  {
    loopTime = millis();
    pid->error = pid->get_target() - pid->get_real();
    pid->integral += pid->error;
    pid->derivative = pid->error - pid->prev_error;

   if(pid->integral > 50/pid->kI)
      pid->integral = 50/pid->kI;

    if(pid->error == 0)
      pid->integral = 0;

    pid->prev_error = pid->error;

    pid->output = (pid->error*pid->kP) + (pid->integral*pid->kI) + (pid->derivative*pid->kD);

    if(pid->output>127)
    {
      pid->output = 127;
    }
    if(pid->output<-127)
    {
      pid->output = -127;
    }

    for(int* m = pid->motors; *m != 0; m++)
    {
      motorSet(abs(*m), pid->output*(*m/abs(*m)));
    }
    
    taskDelayUntil(&loopTime,MOTOR_REFRESH_TIME);
  }
}


PID* init_PID(float (*real)(),float (*target)())
{
  PID *ret = malloc(sizeof(PID)); 
  PID n = {.get_real = real, .get_target = target, .motors = {0}};
  *ret = n;
  return ret;
}

void start_PID(PID *pid)
{
  pid->task = taskCreate(pid_func, TASK_DEFAULT_STACK_SIZE, &pid, TASK_PRIORITY_DEFAULT);
}

float get_PID_output(PID *pid)
{
  return pid->output;
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
  return (abs(pid->error)<pid->stable_delta);
}
