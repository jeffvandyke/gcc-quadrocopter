// TestCompass

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

struct Motors
{
   Motors(Motor *motor1, Motor *motor2, Motor *motor3, Motor *motor4)
   {
      this->m1 = motor1;
      this->m2 = motor2;
      this->m3 = motor3;
      this->m4 = motor4;
   }

   Motor *m1;
   Motor *m2;
   Motor *m3;
   Motor *m4;
};

struct TestParameters
{
   TestParameters()
   {
      kP = 0.0;
      kI = 0.0;
      kD = 0.0;
      gyroWeight = 0.97;
   }

   TestParameters(float _kP, float _kI, float _kD, float _gyroWeight)
   {
      kP = _kP;
      kI = _kI;
      kD = _kD;
      gyroWeight = _gyroWeight;
   }

   float kP;
   float kI;
   float kD;
   float gyroWeight;
};

class Test
{
public:
   Test(struct Motors *motors, OrientationManager *orientationManager, ControllerManager *controllerManager, Bluetooth *bluetooth, ITG3200 *gyro, ADXL345 *accel, HMC5883L *mag);

   // various tests - not all of them are still working but they can be used
   // as a basis or guideline for future testing
   void RPYTest();
   void GyroTest();
   void AccelTest();
   void MagTest();

   void RollTest();
   void YawTest();

   void PitchAxisTest();
   void PitchAxisTestWithMotorValues();

   void PitchAxis();
   void RollPitchTest();
   void AltitudeTest();
   void RollPitchYawTest();

   // best test to use for untethered flight
   void FullFlightTest();

   void yawOnStandTest();

   void debug();
   void debug2();

private:
   void handleMessage(String message);
   void waitForStartCommand();
   void shutdownSequence();
   void startupSequence();

   struct Motors *motors;
   OrientationManager *orientationManager;
   ControllerManager *controllerManager;
   Bluetooth *bluetooth;
   ITG3200 *gyro;
   ADXL345 *accel;
   HMC5883L *mag;
};

#endif

