#ifndef DMPU
#include "DroneMPU.h"
#define DMPU
#endif

MPU::MPU ()
{
   gyro_frequency = IMU.gyroscopeSampleRate();
   acc_frequency = IMU.accelerationSampleRate();
   gyro_last_update = 0;
   acc_last_update = 0;
}


void MPU::GyroscopeCalibration (float precision)
{
  /*calibrates the gyroscope with an error of "precision" per minute*/
  int i;
  vector3D d, err;
  //float dx, dy, dz;
  //float x_err = 0, y_err = 0, z_err = 0;
  float sec = 15;

  do
  {
    for (i = 0; i < sec * gyro_frequency;)
    {
      if (IMU.gyroscopeAvailable())
      {
        IMU.readGyroscope(d.x, d.y, d.z);
        i++;

        err.x += d.x - offset.roll;
        err.y += d.y - offset.pitch;
        err.z += d.z - offset.yaw;
      }
    }
    err.x /= float(i);
    err.y /= float(i);
    err.z /= float(i);

    offset.roll += err.x;
    offset.pitch += err.y;
    offset.yaw += err.z;
  }while (abs(x_err*60/sec) > precision);

  return;  
}


void MPU::AccelerometerCalibration (float precision)
{
  /*calibrates the accelerometer by eliminating the gravitational acceleration with an error of "precision" per minute*/
  int i;
  //float ax, ay, az;
  vector3D a;
  float a_err = 0;
  float sec = 10;
  
  do
  {
    for (i = 0; i < sec * acc_frequency;)
    {
      if (IMU.accelerationAvailable())
      {
        IMU.readAcceleration(a.x, a.y, a.z);
        i++;
        a_err += a.module() - offset.thrust;
      }
    }
    a_err /= float(i);    
    offset.thrust += a_err;
  }while (abs(a_err*60/sec) > precision);

  return;    
}


const SpatialStatus& MPU::UpdateStatus()
{
  /*updates the current status by reading the IMU and computing the integral of its outputs*/
  
  /*the sampling frequency is updated for every call of this function
   *returns the current status*/
  
  vector3D dg;
  vector3D a;
  unsigned long temp;
  
  if (IMU.gyroscopeAvailable())
  {
    IMU.readGyroscope(dg.x, dg.y, dg.z);
    current_status.roll += (dg.x - offset.roll)/gyro_frequency;
    current_status.pitch += (dg.y - offset.pitch)/gyro_frequency; 
    current_status.yaw += (dg.z - offset.yaw)/gyro_frequency;

    temp = millis();

    if (gyro_last_update != 0)
      gyro_frequency = 1/(gyro_last_update - temp);

    gyro_last_update = temp;
  }
  
  if (IMU.accelerationAvailable())
  {
    IMU.readAcceleration(a.x, a.y, a.z);
    a.rotate(current_status.roll/180*PI, current_status.pitch/180*PI, current_status.yaw/180*PI);
    current_status.thrust += (a.z - offset.thrust)/acc_frequency;

    temp = millis();

    if (acc_last_update != 0)
      acc_frequency = 1/(acc_last_update - temp);

    acc_last_update = temp;
  }

  return current_status;
}


float vector3D::module()
{
  return float(sqrt(x*x + y*y + z*a));
}

void vector3D::rotate(float rot_x, float rot_y, float rot_z)
{
  /*rotates a vector in the three axis*/
  this->rotate_x (rot_x);
  this->rotate_y (rot_y);
  this->rotate_z (rot_z);
  return;
}


void vector3D::rotate_x (float rot_x)
{
  /*rotates a vector in the x axis*/
  float ty = y;
  float tz = z;
  y = (ty)*cos(rot_x) - (tz)*sin(rot_x);
  z = (ty)*sin(rot_x) + (tz)*cos(rot_x);
  return;
}


void vector3D::rotate_y (float rot_y)
{
  /*rotates a vector in the y axis*/
  float tx = x;
  float tz = z;
  x = (tx)*cos(rot_y) + (tz)*sin(rot_y);
  z = -(tx)*sin(rot_y) + (tz)*cos(rot_y);
  return;
}

void vector3D::rotate_z (float rot_z)
{
  /*rotates a vector in the z axis*/
  float tx = x;
  float ty = y;
  x = (tx)*cos(rot_z) - (ty)*sin(rot_z);
  y = (tx)*sin(rot_z) + (ty)*cos(rot_z);
  return;
}
