#ifndef DBLYNK
#include "DroneBlynk.h"
#define DBLYNK
#endif


/*da controllare*/
BLYNK_WRITE(V1)
{
  /*thrust*/
  D.input.thrust = float(param.asInt());
}

BLYNK_WRITE(V2)
{
  /*yaw in degrees*/
  D.input.yaw = float(param.asInt());
}

BLYNK_WRITE(V3)
{
  /*pitch in degrees*/
  D.input.pitch = float(param.asInt());
}

BLYNK_WRITE(V4)
{
  /*roll in degrees*/
  D.input.roll = float(param.asInt());
}
