#pragma once
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

class Barometer
{
public:
	Barometer(void);
	~Barometer(void);
	Adafruit_BMP085_Unified barom;
	int setup(void);
	int readRawAltitude(void);
};

