/*
==============================================================
PIDcontrol Library
==============================================================
 Original Code (C) 2015 Cornelius Kloosterman

 This is a class to calculateadjustments to the motors
 
 This is the header file.
==============================================================
*/

#include "Kalman.h"

#ifndef PID_h
#define PID_h

class PIDController
{
public:
   PIDController(int p, int i, int d);
   ~PIDController();
   void move(int deltaPos);
   void setSetPoint(int newPoint);

   	int PD(int currentValue);
	int PI(int currentValue);
	int PID(int currentValue);
	int PID(int currentValue, int dTerm);
private:
	int setPoint;
	int integralSumTerm;
	int lastValue;
	int kp, kd, ki;
	int deltaTime, previousMillis;
	int dTerm, error;
}

#endif