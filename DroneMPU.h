#include <Arduino_LSM6DS3.h>

class SpatialStatus
{
  public:
  SpatialStatus() { roll = 0; pitch = 0; yaw = 0; thrust = 0; }

  float roll;
  float pitch;
  float yaw;
  float thrust;

};

class vector3D
{
  public:
  float x;
  float y;
  float z;

  vector3D() { x = 0; y = 0; z = 0; };
  vector3D(float nx, float ny, float nz) { x = nx; y = ny; z = nz; };

  float module();
  void rotate(float rot_x, float rot_y, float rot_z);
  void rotate_x (float rot_x);
  void rotate_y (float rot_y);
  void rotate_z (float rot_z);
};

class MPU
{
  public:
  MPU();
  
  const SpatialStatus& UpdateStatus();
  const SpatialStatus& CurrentStatus() { return current_status; }
  void GyroscopeCalibration (float precision);
  void AccelerometerCalibration (float precision);

  private:
  SpatialStatus current_status;
  SpatialStatus offset;

  float gyro_frequency;
  float acc_frequency;
  unsigned long gyro_last_update;
  unsigned long acc_last_update;
  
};
