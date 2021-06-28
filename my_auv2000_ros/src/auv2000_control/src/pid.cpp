#include "pid.h"

PID::PID()
{
  resetPID();
}

void PID::runPID()
{
  P_part = Kp * error;
  I_part = Ki * Ts / 2 * e_sum;
  D_part = Kd / Ts * (error - pre_error) ;
  output = P_part + I_part + D_part;
  pre_pre_error = pre_error;
  pre_error = error;
  e_sum += error;
  output = (output > upper_bound) ? upper_bound : output;
  output = (output < lower_bound) ? lower_bound : output;
}

void PID::resetPID()
{
  e_sum = 0;
  error = 0;
  pre_error = 0;
  pre_pre_error = 0;
  output = 0;
}
