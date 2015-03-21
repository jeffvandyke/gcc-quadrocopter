#pragma once
#include <I2C.h>
#include "Accelerometer.h"
#include "Gyroscope.h"
#include "Barometer.h"
#include "Compass.h"
#include "GPS.h"
#include "Trig.h"
//#include <HardwareSerial.h>

enum SensorMode {
	ACCEL_X,
	ACCEL_Y,
	ACCEL_Z,
	GYRO_X,
	GYRO_Y,
	GYRO_Z,
	COMP_X,
	COMP_Y,
	COMP_Z,
	BAROMETER,
	ALL
};

class Quad
{
public:
	Quad(void);
	~Quad(void);


	I2C i2c;
	//int dataBuffer[1000];
	SensorMode sensorMode;

	Accelerometer acc;
	Gyroscope gyro;
	Barometer bar;
	Compass comp;
	GPS gps;
	Trig trig;

	int executeCycle(void);
	int setup(void);
private:
	
	void readSerialCommand(void);
};
