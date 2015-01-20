
#ifndef _TESTCOMPASS_h
#define _TESTCOMPASS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "OrientationManager.h"
#include "ControllerManager.h"
#include "Motor.h"
#include "Bluetooth.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"
#include "string.h"
#include "Test.h"


class compassTest
{
public:
   compassTest(struct Motors *motors, OrientationManager *orientationManager, ControllerManager *controllerManager, Bluetooth *bluetooth, HMC5883L *mag);

   // various tests - not all of them are still working but they can be used
   // as a basis or guideline for future testing

private:
   void handleMessage(String message);
   void waitForStartCommand();
   void shutdownSequence();
   void setSpeed(int speed);

   struct Motors *motors;
   OrientationManager *orientationManager;
   ControllerManager *controllerManager;
   Bluetooth *bluetooth;
   HMC5883L *mag;
};

#endif

