#ifndef DPID
#include "DronePID.h"
#define DPID
#endif

float PID::NextOutput (float error)
{
  /*implementation of a PID controller*/
  
  /*integral*/
  integral_error += error * (float(sample_time)/1000);

  /*derivative*/
  float derivative_error = (error - last_error) / (float(sample_time)/1000);
  last_error = error;
  
  return k_prop*error + k_integ*(integral_error) + k_deriv*derivative_error;
}
