#include "Accelerometer.h"

Accelerometer::Accelerometer(void)
{
	
}


Accelerometer::~Accelerometer(void)
{
}


int Accelerometer::readRawX(void)
{
	return accel.getX();
}


int Accelerometer::readRawY(void)
{
	return accel.getY();
}


int Accelerometer::readRawZ(void)
{
	return accel.getZ();
}


int Accelerometer::setup(void)
{
	//Serial1.println("accelometer setup");
	accel.begin();
	return 0;
}



