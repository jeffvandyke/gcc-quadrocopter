/*
==============================================================
 Trig Library
==============================================================
 Original Code (C) 2015 Cornelius Kloosterman

 This is a library to calculateadjustments to the motors
 
 This is the header file.
==============================================================
*/

#include "Kalman.h"
using namespace std;

#ifndef PID_h
#define PID_h

class PIDController
{
public:
   PIDController();
   ~PIDController();
   void move(quadState_t deltaPos)
   quadState_t PIDUpdate(quadState_t newVal);
private:
	quadState_t setPoint;
	quadState_t correctionFactor;
	int kI, kD, kP;
	int deltaTime;
	int PD(int value);
	int PI(int value, int* integralSumTerm);
	int PID(int value, int* integralSumTerm);
}

#endif