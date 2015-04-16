#pragma once
#include <I2C.h>
#include "Accelerometer.h"
#include "Gyroscope.h"
#include "Barometer.h"
#include "Compass.h"
#include "GPS.h"
#include "Trig.h"
#include "Kalman.h"
//#include <HardwareSerial.h>

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
	Kalman Filter;
	Trig trig;

	int executeCycle(void);
	int setup(void);
	int motorInitialize(void);
	int motorSet(void);
	int getSensorVals(void);
	int getGPSval(void);
	int findSensorBias(void);


	//bitshifted ints to be passed to the Kalman filter
	int compX, compY, compZ, gyroX, gyroY, gyroZ, accX, accY, accZ, latitude, longitude;
	quadState_T quadState;
	int dutyCycle1, dutyCycle2, dutyCycle3, dutyCycle4;

	void readSerialCommand(void);
};
