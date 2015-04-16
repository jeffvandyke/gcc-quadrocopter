#include "Gyroscope.h"

Gyroscope::Gyroscope(void)
{
	
}


Gyroscope::~Gyroscope(void)
{
}


int Gyroscope::readRawX(void)
{
	return gyro.getRotationX();
}


int Gyroscope::readRawY(void)
{
	return gyro.getRotationY();
}


int Gyroscope::readRawZ(void)
{
	return gyro.getRotationZ();
}


int Gyroscope::setup(void)
{
	gyro.initialize();
	return 0;
}



