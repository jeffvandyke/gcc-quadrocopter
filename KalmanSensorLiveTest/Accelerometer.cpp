#include "Accelerometer.h"

Accelerometer::Accelerometer(void)
{
	
}


Accelerometer::~Accelerometer(void)
{
}


inline int Accelerometer::readRawX(void)
{
	return accel.getX();
}


inline int Accelerometer::readRawY(void)
{
	return accel.getY();
}


int Accelerometer::readRawZ(void)
{
	return accel.getZ();
}


int Accelerometer::setup(void)
{
	Serial.println("accelometer setup");
	accel.begin();
	return 0;
}
