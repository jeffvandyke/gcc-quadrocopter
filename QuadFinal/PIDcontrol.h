/*
==============================================================
PIDcontrol Library
==============================================================
 Original Code (C) 2015 Cornelius Kloosterman

 This is a class to calculateadjustments to the motors
 
 This is the header file.
==============================================================
*/
#include <Arduino.h>
#include "Kalman.h"

#ifndef PID_h
#define PID_h

class PIDController
{
public:
	PIDController();
	void changeGain(float p, float i, float d);
	~PIDController();
	void move(int deltaPos);
	void setSetPoint(float newPoint);

	float PID(float currentValue);
	float PID(float currentValue, float dTerm);
private:
	float setPoint;
	float integralSumTerm;
	float lastValue;
	float kP, kD, kI;
	float deltaTime, previousMillis;
	float dTerm, error;
};

#endif



