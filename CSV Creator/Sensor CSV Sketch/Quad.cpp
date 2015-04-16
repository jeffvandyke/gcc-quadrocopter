#include "Quad.h"
#include <Wire.h>
#include <I2C.h>
#include "Bluetooth.h"

#define MOTOR_OUT 6
#define MOTOR_TRIG 53
#define OUTPIN 6
#define TRIGPIN 53
#define PWMax 110

Quad::Quad(void)
{
	i2c = I2C();
	return;
}

Quad::~Quad(void)
{
}

int Quad::motorInitialize(void)
{
	analogWrite(2,0);
	analogWrite(3,0);
	analogWrite(5,0);
	analogWrite(6,0);
	delay(2000);
	analogWrite(2,PWMax);
	analogWrite(3,PWMax);
	analogWrite(5,PWMax);
	analogWrite(6,PWMax);
	delay(2000);
	analogWrite(2,0);
	analogWrite(3,0);
	analogWrite(5,0);
	analogWrite(6,0);
	delay(2000);

	return 0;
}

int Quad::motorSet(void)
{
  analogWrite(2,PWMax+25);
  analogWrite(3,PWMax+25);
  analogWrite(5,PWMax+25);
  analogWrite(6,PWMax+25);
  delay(2000);
  analogWrite(2,0);
  analogWrite(3,0);
  analogWrite(5,0);
  analogWrite(6,0);
  delay(2000);


	return 0;
}

int Quad::setup(void)
{
	sensorMode = ALL;
	i2c.initialize();
	Serial.begin(9600);
//	Serial.print("IMUTester::setupTester called");
	acc.setup();
	gyro.setup();
	bar.setup();
	comp.setup();
	//gpssetup {
	Serial2.begin(4800);
	delay(1000);
	//}
	return 0;
}

int Quad::executeCycle(void)
{

	this->readSerialCommand();

	while (Serial2.available())
     {
		char c = Serial2.read();
		if(gps.encode(c))
			break;
	}

	getSensorVals();
	getGPSval();

	delay(10);
	return 0;
}

int getSensorVals(void)
{
	//Accelerometer	
	accX=	1024*acc.readRawX();
	accY=	1024*acc.readRawY();
	accZ=	1024*acc.readRawZ();
	//Gyroscope
	gyroX=	1024*gyro.readRawX();
	gyroY=	1024*gyro.readRawY();
	gyroZ=	1024*gyro.readRawZ();
	//Compass
	comX=	1024*comp.getRawX();
	comY=	1024*comp.getRawY();
	comZ=	1024*comp.getRawZ();
}

int getGPSval(void)
{
	latitude=	1024*gps.readRawLat();
	longitude=	1024*gps.readRawLong();
}

//calculates sensor bias at startup
int findSensorBias(void)
{
};

void Quad::readSerialCommand(void) {
	int serialData = Serial.read();

	if (serialData != -1)
		// clear some space
			Serial.print("\n\n\n");

	switch (serialData)
	{
	case (int)'1':
		motorSet(100);
		break;
	case (int)'2':
		motorSet(120);
		break;
	case (int)'3':
		motorSet(130);
		break;
	case (int)'4':
		motorSet(140);
		break;
	case (int)'5':
		motorSet(150);
		break;
	case (int)'6':
		motorSet(160);
		break;
	case (int)'7':
		motorSet(170);
		break;
	case (int)'8':
		motorSet(180);
		break;
	case (int)'9':
		motorSet(190);
		break;
	case (int)'10':
		motorSet(200);
		break;
	case (int)'11':
		motorSet(210);
		break;
	case (int)'12':
		motorSet(220);
		break;
	case -1:
		break;
	default:
		Serial.print("unknown command: ");
		Serial.println((char)serialData);
	}
}