#pragma once
#include "I2C.h"
//#include "Accelerometer.h"
#include "Gyroscope.h"
#include "Barometer.h"
#include "Compass.h"
#include "GPS.h"

#include "Adafruit_Sensor.h"
#include "Adafruit_ADXL345_U.h"

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



	Adafruit_ADXL345_Unified accel;
	ITG3200 gyro;


	I2C i2c;
	SensorMode sensorMode;

//	Accelerometer acc;
//	Gyroscope gyro;
	Barometer bar;
	Compass comp;
	GPS gps;

	long newLong,newLat;
	unsigned long fix_age;
	unsigned long date, time, oldTime;

	unsigned long finish;
	unsigned long stopTime;

	int cycles;
	unsigned long sumCollectionTime;
	unsigned long sensorCollectionStart;


	int executeCycle(void);
	int setupTester(void);

	bool reportGyro;
	bool reportGyroAll;
	bool reportAccel;
	bool reportComp;
	bool reportGPS;

	//void initialize(int);

	void printGPS(void);
	int timeGyro(void);
	int timeGyroAll(void);
	int timeAccel(void);
	int timeComp(void);

	void serialPrint(String s);
	void serialPrint(int i);
	void serialPrint(float i);
	void serialPrintln(String s);
	void serialPrintln(int i);

private:

	void readSerialCommand(void);
};

