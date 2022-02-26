#ifndef DLIB
#include "DroneLibrary.h"
#define DLIB
#endif

void Drone::SetPins(unsigned BLPin, unsigned BRPin, unsigned FRPin, unsigned FLPin)
{  
  back_left_motor.attach(BLPin, MIN_PW, MAX_PW);
  back_right_motor.attach(BRPin, MIN_PW, MAX_PW);
  front_right_motor.attach(FRPin, MIN_PW, MAX_PW);
  front_left_motor.attach(FLPin, MIN_PW, MAX_PW);
  return;
}

void Drone::CalibrateSensors (float gyro_prec, float acc_prec)
{
  sensors.GyroscopeCalibration(gyro_prec);
  sensors.AccelerometerCalibration(acc_prec);
  return;
}

void Drone::SetRollParameters(float kp, float ki, float kd)
{
  roll_controller.k_prop = kp;
  roll_controller.k_intg = ki;
  roll_controller.k_deriv = kd;
  return;
}

void Drone::SetPitchParameters(float kp, float ki, float kd)
{
  pitch_controller.k_prop = kp;
  pitch_controller.k_intg = ki;
  pitch_controller.k_deriv = kd;
  return;
}

void Drone::SetYawParameters(float kp, float ki, float kd)
{
  yaw_controller.k_prop = kp;
  yaw_controller.k_intg = ki;
  yaw_controller.k_deriv = kd;
  return;
}

void Drone::SetThrustParameters(float kp, float ki, float kd)
{
  thrust_controller.k_prop = kp;
  thrust_controller.k_intg = ki;
  thrust_controller.k_deriv = kd;
  return;
}

void Drone::run()
{
  SpatialStatus current_status = sensors.UpdateStatus();
 
  ControlRoll (current_status.roll - input.roll);
  ControlPitch (current_status.pitch - input.pitch;);
  ControlYaw (current_status.yaw - input.yaw);
  ControlThrust (current_status.thrust - input.thrust);
  return;
}

/*void Drone::UpdateStatus ()
{
  const SpatialStatus current_status = sensors.UpdateStatus();
  
  roll_control.error = current_status.roll - input.roll;
  pitch_control.error = current_status.pitch - input.pitch;
  yaw_control.error = current_status.yaw - input.yaw;
  thrust_control.error = current_status.thrust - input.thrust;
  return;
}

void Drone::PIDControl()
{
  ControlRoll();
  ControlPitch();
  ControlYaw();
  ControlThrust();
  return;
}*/

void Drone::ControlRoll (float error)
{
  float s = roll_controller.NextOutput(error);

  back_left_motor.write( back_left_motor.read() - s/4);
  back_right_motor.write( back_left_motor.read() + s/4);   //  -  +
  front_right_motor.write( back_left_motor.read() + s/4);  //  -  +
  front_left_motor.write( back_left_motor.read() - s/4);
  
  return;
}
 
void Drone::ControlPitch(float error)
{
  float s = pitch_controller.NextOutput(error);

  back_left_motor.write( back_left_motor.read() - s/4);
  back_right_motor.write( back_left_motor.read() - s/4);   //  +  +
  front_right_motor.write( back_left_motor.read() + s/4);  //  -  -
  front_left_motor.write( back_left_motor.read() + s/4);
  
  return;
}

void Drone::ControlYaw(float error)
{
  float s = yaw_controller.NextOutput(error);

  back_left_motor.write( back_left_motor.read() + s/4);
  back_right_motor.write( back_left_motor.read() - s/4);   //  -  +
  front_right_motor.write( back_left_motor.read() + s/4);  //  +  -
  front_left_motor.write( back_left_motor.read() - s/4);
  
  return;
}
void Drone::ControlThrust(float error)
{
  float s = thrust_controller.NextOutput(error);

  back_left_motor.write( back_left_motor.read() + s/4);
  back_right_motor.write( back_left_motor.read() + s/4);  
  front_right_motor.write( back_left_motor.read() + s/4);  
  front_left_motor.write( back_left_motor.read() + s/4);
  
  return;
}
