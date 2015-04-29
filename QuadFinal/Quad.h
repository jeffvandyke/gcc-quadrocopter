#pragma once
#include "I2C.h"
#include "Accelerometer.h"
#include "Gyroscope.h"
#include "Barometer.h"
#include "Compass.h"
#include "Bluetooth.h"
#include "GPS.h"
//#include "Trig.h"
#include "Kalman.h"
#include "PIDcontrol.h"
#include "math.h"
//#include <HardwareSerial.h>


class Quad
{
public:
	Quad(void);
	~Quad(void);


	I2C i2c;
	//int dataBuffer[1000];

	Bluetooth blue;

	Accelerometer acc;
	Gyroscope gyro;
  	Barometer bar;
	Compass comp;
	GPS gps;
	KalmanFilter Filter;

    PIDController xPosition, yPosition, zPosition;

    PIDController xAngle, yAngle, zAngle;

	int executeCycle(void);
	int setup(void);
	int motorInitialize(void);
	int motorSet(int, int);
	int getSensorVals(void);
//	int getGPSval(void);
	int findSensorBias(void);
	int adjustMotors(void);
	int adjustMotors(int);
	void readSerialCommand(void);

	int waitFor();

	//ints to be passed to the Kalman filter
	int compX, compY, compZ,
		gyroX, gyroY, gyroZ,
		accX, accY, accZ,
        barZ,
		GPSlat, GPSlong, GPSalt;
	bool readGPS;

	//Correction factors
	int xPosCorrect, yPosCorrect, zPosCorrect,
		xAngCorrect, yAngCorrect, zAngCorrect;

	//Biases calculated at startup
	int gyroXBias, gyroYBias, gyroZBias,
		gpsLatBias, gpsLongBias, gpsAltBias;

	//

	quadState_t quadState;
	int dutyCycle1, dutyCycle2, dutyCycle3, dutyCycle4,
		m1speed, m2speed, m3speed, m4speed;
	//loopTime stores the time since the waitFor function was run.
	//controlTime stores the time since th
	int loopTime;

	//void readSerialCommand(void);
};



