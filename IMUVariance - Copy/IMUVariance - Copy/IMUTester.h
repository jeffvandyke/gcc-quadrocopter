#pragma once
#include <I2C.h>
#include "Accelerometer.h"
#include "Gyroscope.h"
#include "Barometer.h"
#include "Compass.h"
#include "GPS.h"

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

class IMUTester
{
public:
	IMUTester(void);
	~IMUTester(void);


	I2C i2c;
	//int dataBuffer[1000];
	SensorMode sensorMode;

	Accelerometer acc;
	Gyroscope gyro;
	Barometer bar;
	Compass comp;
	GPS gps;
	trig trig;

	int executeCycle(void);
	int setupTester(void);
private:
	
	void readSerialCommand(void);
};

