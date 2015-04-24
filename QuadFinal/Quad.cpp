#include "Quad.h"
#include <Wire.h>
#include "I2C.h"
#include "Bluetooth.h"

#define MOTOR_OUT 6
#define MOTOR_TRIG 53
#define OUTPIN 6
#define TRIGPIN 53
#define PWMIN 110

#define MAX_MOTOR_SPEED 128
#define MAX_ANGULAR_DEFLECTION 20
#define MIN_ANGULAR_DEFLECTION -20
#define MAX_ALTITUDE_CORRECTION 100

#define MOTOR1 2
#define MOTOR2 3
#define MOTOR3 5
#define MOTOR4 6

Quad::Quad(void)
{
	i2c = I2C();
	loopTime=millis();
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

int Quad::motorSet(int motor, int speed)
{
	speed	=	floor(1.28 * value) + PWMIN;

	analogWrite(motor, speed);

//	delay(2000);
	analogWrite(motor, 0);
//	delay(2000);


	return 0;
}

int Quad::setup(void)
{
	sensorMode = ALL;
	i2c.initialize();
	Serial.begin(9600);
	acc.setup();
	gyro.setup();
	bar.setup();
	comp.setup();

	//gpssetup
	Serial2.begin(4800);
	delay(1000);

	findSensorBias();

    // ordinarily, these would be the average of the gyro, acc, gps axis
    // over several samples, but these may be just fine for our purposes.
    Filter.initialize( 6.9158f, 9.9410f, 21.515f,
            -16.027f, 0.9157f, 0.6185f,
            //         -12504.f, -13316.f, -24942.f );
        // gps = bogus values, so for now, bias is zero
        0,0,0);

	return 0;
}

// global counter, UNTESTED
int nIteration = 0;

int Quad::executeCycle(void)
{

    nIteration++;
	this->readSerialCommand();

	this->getSensorVals();
	readGPS = getGPSval();

	void Filter.assignSensorValues(
		accX, accY, accZ,	// acceleration
		gyroX, gyroY, gyroZ,	// gyroscope

        // !!! bogus values for now, reads true north from compass,
        // and static position for GPS

        0,100, // (compass)
        0,0,0, // (GPS)

		// // compX, compY, 	// Compass
		// // GPSlat, GPSlong, GPSalt,	// GPS

        nIteration % 10 == 0);
		//readGPS); // use gps value

	Filter.predictAndUpdate();

	//wait until a constant time has passed.
	waitFor();

	//retrieve values from Kalman Filter
	quadState = Filter.getQuadState();

	//calculate correctionfactors for position. X and Z are used to change the setpoint of the angular deflection
	//the correction factors will be altered by the proportional gain to have values similar to degrees*1024.
	xPosCorrect = xPosition.PID(quadState.xPosition, quadState.xVelocity);
	yPosCorrect = yPosition.PID(quadState.yPosition, quadState.yVelocity);
	zPosCorrect = zPosition.PID(quadState.zPosition, quadState.zVelocity);
	

	//Our design spec doesn't allow for angular deflection greater than 20 degrees.
	if(xPosCorrect	>	MAX_ANGULAR_DEFLECTION)
		xPosCorrect	=	MAX_ANGULAR_DEFLECTION;
	if(xPosCorrect	<	-MIN_ANGULAR_DEFLECTION)
		xPosCorrect	=	-MIN_ANGULAR_DEFLECTION;
	if(yPosCorrect	>	MAX_ANGULAR_DEFLECTION)
		yPosCorrect	=	MAX_ANGULAR_DEFLECTION;
	if(yPosCorrect	<	-MIN_ANGULAR_DEFLECTION)
		yPosCorrect	=	-MIN_ANGULAR_DEFLECTION;

	//we also don't want our quad to attempt to fly too quickly upwards, either
	if(zPosCorrect	>	MAX_ALTITUDE_CORRECTION)
		zPosCorrect	=	MAX_ALTITUDE_CORRECTION;

	xAngle.setSetPoint(xPosCorrect);
	yAngle.setSetPoint(yPosCorrect);

	//With the corrected setpoint
    xAngCorrect	= xAngle.PID(quadState.xAngle, quadState.xRotation);
	yAngCorrect = yAngle.PID(quadState.yAngle, quadState.yRotation);
	zAngCorrect = zAngle.PID(quadState.zAngle, quadState.zRotation);

	delay(10);
	return 0;
}

//updates all of the sensor values stored in the Quad object
int Quad::getSensorVals(void)
{
	//Accelerometer	
	accX=	acc.readRawX();
	accY=	acc.readRawY();
	accZ=	acc.readRawZ();
	//Gyroscope
	gyroX=	gyro.readRawX()-gyroXBias;
	gyroY=	gyro.readRawY()-gyroYBias;
	gyroZ=	gyro.readRawZ()-gyroZBias;
	//Compass
	comX=	comp.getRawX();
	comY=	comp.getRawY();
	comZ=	comp.getRawZ();
}

//updates the internal GPS values
bool Quad::getGPSval(void)
{
	//waits until a full signal is read from the gps
	while (Serial2.available())
     {
		char c = Serial2.read();
		if(gps.encode(c))
			break;
	}

	//since there is no way to tell how stale the value that the GPS holds is, this function merely checks to see if
	//its a new value or not
	if gps.readRawLat == GPSlat;
		return false;

	GPSlat	=	gps.readRawLat()-gpsLatBias;
	GPSlong	=	gps.readRawLong()-gpsLongBias;
	GPSalt	=	gps.readRawAlt()-gpsAltBias;
	return true;
}

//calculates sensor bias at startup
int Quad::findSensorBias(void)
{
	//arrays for collecting samples
	int gpsLatSample[8];
	int gpsLongSample[8];
	int gpsAltSample[8];

	int gyroXSample[8];
	int gyroYSample[8];
	int gyroZSample[8];

	for(int i;i<8;i++)
	{
		//waits until new value comes from GPS object
		do
		{
			while (Serial2.available())
			 {
				char c = Serial2.read();
				if(gps.encode(c))
					break;
			}
		}while(gpsLatSample[i-1] != gps.readRawLat && i != 0);

		//Samples gyro and GPS whenever its relevant
		gpsLatSample[i]		=	gps.readRawLat();
		gpsLongSample[i]	=	gps.readRawLong();
		gpsAltSample[i]		=	gps.readRawAlt();

		gyroXSample[i]		=	gyro.readRawX();
		gyroYSample[i]		=	gyro.readRawY();
		gyroZSample[i]		=	gyro.readRawZ();
	}

	//take average of gps and gyro samples to determine bias
		for( k = 0; k < 8; k++ ) { gpsLatBias += gpsLatSample[k]; }
		gpsLatBias = gpsLatBias / 8;

		for( k = 0; k < 8; k++ ) { gpsLongBias += gpsLongSample[k]; }
		gpsLongBias = gpsLongBias / 8;

		for( k = 0; k < 8; k++ ) { gpsAltBias += gpsAltSample[k]; }
		gpsAltBias = gpsAltBias / 8;

		for( k = 0; k < 8; k++ ) { gyroXBias += gyroXSample[k]; }
		gyroXBias = gyroXBias / 8;

		for( k = 0; k < 8; k++ ) { gyroYBias += gyroYSample[k]; }
		gyroYBias = gyroYBias / 8;

		for( k = 0; k < 8; k++ ) { gyroZBias += gyroZSample[k]; }
		gyroZBias = gyroZBias / 8;
}

int Quad::adjustMotors(void)
{
	// adjust for pitch and roll
	m1speed += xAngCorrect;
    m3speed -= xAngCorrect;

	m2speed -= yAngCorrect;
	m4speed += yAngCorrect;

	//adjust for altitude
	m1speed += altitudeCorrection;
	m2speed += altitudeCorrection;
	m3speed += altitudeCorrection;
	m4speed += altitudeCorrection;

	// adjust for yaw
	m1speed -= yawCorrection;
	m3speed -= yawCorrection;

	m2speed += zAngCorrect;
	m4speed += zAngCorrect;

	if(m1speed	>	MAX_MOTOR_SPEED)
		m1speed	=	MAX_MOTOR_SPEED;
	if(m2speed	>	MAX_MOTOR_SPEED)
		m2speed	=	MAX_MOTOR_SPEED;
	if(m3speed	>	MAX_MOTOR_SPEED)
		m3speed	=	MAX_MOTOR_SPEED;
	if(m3speed	>	MAX_MOTOR_SPEED)
		m3speed	=	MAX_MOTOR_SPEED;

	motorSet(MOTOR1, m1speed);
	motorSet(MOTOR2, m2speed);
	motorSet(MOTOR3, m3speed);
	motorSet(MOTOR4, m4speed);
}

int Quad::waitFor()
{
	delay(10 - (millis()-loopTime))
}

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



