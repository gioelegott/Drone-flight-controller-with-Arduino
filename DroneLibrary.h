
#ifndef DMPU
#include "DroneMPU.h"
#define DMPU
#endif
#ifndef DPID
#include "DronePID.h"
#define DPID
#endif

#include <Servo.h>

class Drone
{
  public:
  SpatialStatus input; //nuova classe se l'input è più complicato
  
  void SetPins(unsigned BLPin, unsigned BRPin, unsigned FRPin, unsigned FLPin);
  void CalibrateSensors (float gyro_prec, float acc_prec);
  
  void SetRollParameters(float kp, float ki, float kd);
  void SetPitchParameters(float kp, float ki, float kd);
  void SetYawParameters(float kp, float ki, float kd);
  void SetThrustParameters(float kp, float ki, float kd);

  void run();
  
  //void UpdateStatus();
  //void PIDControl();
  
  private:  
  Servo back_left_motor;
  Servo back_right_motor;
  Servo front_right_motor;
  Servo front_left_motor;
  const unsigned MIN_PW = 1000;
  const unsigned MAX_PW = 2000;

  MPU sensors;
  //SpatialStatus current_status;
  
  PID roll_controller;
  PID pitch_controller;
  PID yaw_controller;
  PID thrust_controller;
  
  void ControlRoll();
  void ControlPitch();
  void ControlYaw();
  void ControlThrust();

};
