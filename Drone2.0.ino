#include "DroneLibrary.h"
#include "DroneBlynk.h"

char auth[] = "BI6aPgR0OlYiY7tehr9SzInEjSKWzlJ-";
char ssid[] = "AndroidAP";
char pass[] = "";

Drone D;
unsigned BLPin = 11, BRPin = 12, FRPin = 13, FLPin = 14;

void setup()
{
  Serial.begin(9600);
  Blynk.begin(auth, ssid, pass);
  
  D.SetPins(BLPin, BRPin, FRPin, FLPin);
  D.CalibrateSensors(0.03, 0.01);

  D.SetRollParameters(0, 0, 0);
  D.SetPitchParameters(0, 0, 0);
  D.SetYawParameters(0, 0, 0);
  D.SetThrustParameters(0, 0, 0);

  return;
}

void loop()
{
  Blynk.run();
  D.run();
  
  //D.UpdateStatus();
  //D.PIDControl();  

  return;
}
