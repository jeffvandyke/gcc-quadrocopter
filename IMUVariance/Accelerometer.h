#pragma once
#include "Adafruit_Sensor.h"
#include "Adafruit_ADXL345_U.h"

class Accelerometer
{
public:
	Accelerometer(void);
	~Accelerometer(void);

	Adafruit_ADXL345_Unified accel;

	inline int readRawX(void);
	inline int readRawY(void);
	int readRawZ(void);
	int setup(void);

};

