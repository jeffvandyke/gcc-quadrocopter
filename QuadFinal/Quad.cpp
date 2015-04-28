#include "Quad.h"
#include <Wire.h>
#include "I2C.h"
#include "Bluetooth.h"

#define MOTOR_OUT 6
#define MOTOR_TRIG 53
#define OUTPIN 6
#define TRIGPIN 53
#define PWMIN 100

#define MAX_LOOP_TIME 30

#define MAX_MOTOR_SPEED 128
#define MAX_ANGULAR_DEFLECTION 20
#define MIN_ANGULAR_DEFLECTION -20
#define MAX_ALTITUDE_CORRECTION 100

#define MOTOR1 6
#define MOTOR2 3
#define MOTOR3 5
#define MOTOR4 2

//The carefully tuned values for all of our PID control loops.
#define X_POS_KP 0
#define X_POS_KI 0
#define X_POS_KD 0

#define Y_POS_KP 0
#define Y_POS_KI 0
#define Y_POS_KD 0

#define Z_POS_KP 0
#define Z_POS_KI 0
#define Z_POS_KD 0

#define X_ANG_KP 1.6
#define X_ANG_KI 0
#define X_ANG_KD 0.6

#define Y_ANG_KP 1.6
#define Y_ANG_KI 0
#define Y_ANG_KD 0.6

#define Z_ANG_KP 0
#define Z_ANG_KI 0
#define Z_ANG_KD 0

Quad::Quad(void)
{
	i2c = I2C();
	loopTime=millis();

	xPosition.changeGain(X_POS_KP, X_POS_KI, X_POS_KD);
	yPosition.changeGain(Y_POS_KP, Y_POS_KI, Y_POS_KD);
	zPosition.changeGain(Z_POS_KP, Z_POS_KI, Z_POS_KD);

	xAngle.changeGain(X_ANG_KP, X_ANG_KI, X_ANG_KD);
	yAngle.changeGain(Y_ANG_KP, Y_ANG_KI, Y_ANG_KD);
	zAngle.changeGain(Z_ANG_KP, Z_ANG_KI, Z_ANG_KD);

	return;
}

Quad::~Quad(void)
{
}

int Quad::motorInitialize(void)
{
	analogWrite(MOTOR1,0);
	analogWrite(MOTOR2,0);
	analogWrite(MOTOR3,0);
	analogWrite(MOTOR4,0);
	delay(2000);
	analogWrite(MOTOR1,PWMIN);
	analogWrite(MOTOR2,PWMIN);
	analogWrite(MOTOR3,PWMIN);
	analogWrite(MOTOR4,PWMIN);
	delay(2000);
	analogWrite(MOTOR1,0);
	analogWrite(MOTOR2,0);
	analogWrite(MOTOR3,0);
	analogWrite(MOTOR4,0);
	delay(2000);

	return 0;
}

int Quad::motorSet(int motor, int speed)
{
	speed	=	speed + PWMIN;

	analogWrite(motor, speed);


	/*blue.println("motorSet");
	blue.print(motor);
	blue.println("");
	blue.print(speed);
	blue.println("");*/
	return 0;
}

int Quad::setup(void)
{
	i2c.initialize();
	Serial.begin(9600);
	acc.setup();
	gyro.setup();
//	bar.setup();
	comp.setup();
	motorInitialize();
	blue.initialize();

	blue.println("setup");

	//gpssetup
	Serial2.begin(4800);
	delay(1000);

//	findSensorBias();

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

	//this->readSerialCommand();
	if(blue.readLine() == "x")
		exit(1);
	blue.println("!!executeCycle!!");

	getSensorVals();
	//readGPS = getGPSval();

	//Serial1.print(gyroX);
	//blue.println(" :gyroX");
	//Serial1.print(gyroY);
	//blue.println(" :gyroY");
	//Serial1.print(compX);
	//blue.println(" :compX");
	//Serial1.print(compY);
	//blue.println(" :compY");


	Filter.assignSensorValues(
		accX, accY, accZ,	// acceleration
		gyroX, gyroY, gyroZ,	// gyroscope

		// // compX, compY, 	// Compass
		// // GPSlat, GPSlong, GPSalt,	// GPS

        // !!! bogus values for now, reads true north from compass,
        // and static position for GPS

		// compX, compY, 	// Compass
        0,100, // (compass bogus)
		/*GPSlat, GPSlong, GPSalt,*/	// GPS
		 0, 0, 0,
		//readGPS); // use gps value
         nIteration % 10 == 0);

	Filter.predictAndUpdate();

	//retrieve values from Kalman Filter
	quadState = Filter.getQuadState();

	Serial1.print(quadState.xAngle);
	blue.println(" :xAngle");
	Serial1.print(quadState.xRotation);
	blue.println(" :xRotation");
	Serial1.print(quadState.yAngle);
	blue.println(" :yAngle");
	Serial1.print(quadState.yRotation);
	blue.println(" :yRotation");

	//calculate correctionfactors for position. X and Z are used to change the setpoint of the angular deflection
	//the correction factors will be altered by the proportional gain to have values similar to degrees*1024.
//	xPosCorrect = xPosition.PID(quadState.xPosition, quadState.xVelocity);
//	yPosCorrect = yPosition.PID(quadState.yPosition, quadState.yVelocity);
//	zPosCorrect = zPosition.PID(quadState.zPosition, quadState.zVelocity);

	//Our design spec doesn't allow for angular deflection greater than 20 degrees.
	//if(xPosCorrect	>	MAX_ANGULAR_DEFLECTION)
	//	xPosCorrect	=	MAX_ANGULAR_DEFLECTION;
	//if(xPosCorrect	<	-MIN_ANGULAR_DEFLECTION)
	//	xPosCorrect	=	-MIN_ANGULAR_DEFLECTION;
	//if(yPosCorrect	>	MAX_ANGULAR_DEFLECTION)
	//	yPosCorrect	=	MAX_ANGULAR_DEFLECTION;
	//if(yPosCorrect	<	-MIN_ANGULAR_DEFLECTION)
	//	yPosCorrect	=	-MIN_ANGULAR_DEFLECTION;

	////we also don't want our quad to attempt to fly too quickly upwards, either
	//if(zPosCorrect	>	MAX_ALTITUDE_CORRECTION)
	//	zPosCorrect	=	MAX_ALTITUDE_CORRECTION;

	//xAngle.setSetPoint(xPosCorrect);
	//yAngle.setSetPoint(yPosCorrect);

	//With the corrected setpoint
//   xAngCorrect	= xAngle.PID(quadState.xAngle, quadState.xRotation);
	yAngCorrect = yAngle.PID(quadState.yAngle, quadState.yRotation);
//	zAngCorrect = zAngle.PID(quadState.zAngle, quadState.zRotation);

	//adjustMotors();
	adjustMotors(quadState.zAngle);


	//wait until a constant time has passed.
	waitFor();

	delay(10);
	return 0;
}

//updates all of the sensor values stored in the Quad object
int Quad::getSensorVals(void)
{
	blue.println("getSensorVals");

	//Accelerometer
	accX=	acc.readRawX();
	accY=	acc.readRawY();
	accZ=	acc.readRawZ();
	//Gyroscope
	gyroX=	gyro.readRawX()-gyroXBias;
	gyroY=	gyro.readRawY()-gyroYBias;
	gyroZ=	gyro.readRawZ()-gyroZBias;
	//Compass
	compX=	comp.getRawX();
	compY=	comp.getRawY();
	compZ=	comp.getRawZ();
}

//updates the internal GPS values
int Quad::getGPSval(void)
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
	if (gps.readRawLat() == GPSlat)
		return false;

	GPSlat	=	gps.readRawLat()-gpsLatBias;
	GPSlong	=	gps.readRawLong()-gpsLongBias;
	GPSalt	=	gps.readRawAlt()-gpsAltBias;
	return true;
}

//calculates sensor bias at startup
//int Quad::findSensorBias(void)
//{
//	//arrays for collecting samples
//	int gpsLatSample[8];
//	int gpsLongSample[8];
//	int gpsAltSample[8];
//
//	for(int i;i<8;i++)
//	{
//		int k;
//		//waits until new value comes from GPS object
//		do
//		{
//			while (Serial2.available())
//			 {
//				char c = Serial2.read();
//				if(gps.encode(c))
//					break;
//			}
//		}while(gpsLatSample[i-1] != gps.readRawLat() && i != 0);
//
//		//Samples  GPS whenever its relevant
//		gpsLatSample[i]		=	gps.readRawLat();
//		gpsLongSample[i]	=	gps.readRawLong();
//		gpsAltSample[i]		=	gps.readRawAlt();
//
//	}
//		int k;
//	//take average of gps samples to determine bias
//	for( k = 0; k < 8; k++ ) { gpsLatBias += gpsLatSample[k]; }
//	gpsLatBias = gpsLatBias / 8;
//
//	for( k = 0; k < 8; k++ ) { gpsLongBias += gpsLongSample[k]; }
//	gpsLongBias = gpsLongBias / 8;
//
//	for( k = 0; k < 8; k++ ) { gpsAltBias += gpsAltSample[k]; }
//	gpsAltBias = gpsAltBias / 8;
//
//}

int Quad::adjustMotors(void)
{
	blue.println("adjustMotors");
	// adjust for pitch and roll
	m1speed += xAngCorrect;
	m2speed += xAngCorrect;
    m3speed -= xAngCorrect;
	m4speed -= xAngCorrect;

	//m1speed += yAngCorrect;
	//m2speed += yAngCorrect;
    //m3speed -= yAngCorrect;
	//m4speed -= yAngCorrect;

	////adjust for altitude
	//m1speed += zPosCorrect;
	//m2speed += zPosCorrect;
	//m3speed += zPosCorrect;
	//m4speed += zPosCorrect;

	//// adjust for yaw
	//m1speed -= zAngCorrect;
	//m3speed -= zAngCorrect;

	//m2speed += zAngCorrect;
	//m4speed += zAngCorrect;

	if(m1speed	>	MAX_MOTOR_SPEED)
		m1speed	=	MAX_MOTOR_SPEED;
	else if(m1speed < 0)
		m1speed = 0;
	//if(m2speed	>	MAX_MOTOR_SPEED)
	//	m2speed	=	MAX_MOTOR_SPEED;
	//else if(m2speed < 0)
	//	m2speed = 0;
	if(m3speed	>	MAX_MOTOR_SPEED)
		m3speed	=	MAX_MOTOR_SPEED;
	else if(m3speed < 0)
		m3speed = 0;
	//if(m4speed	>	MAX_MOTOR_SPEED)
	//	m4speed	=	MAX_MOTOR_SPEED;
	//else if(m4speed < 0)
	//	m4speed = 0;

	motorSet(MOTOR1, m1speed);
//	motorSet(MOTOR2, m2speed);
	motorSet(MOTOR3, m3speed);
//	motorSet(MOTOR4, m4speed);
}

// motor correction that takes z angle into account
int Quad::adjustMotors(int zAngle)
{
    zAngle -= 45; // for matching this coordinate system
    /*
     *            +y (for 0° Z angle)
     *
     *   4  usb  1
     *    \  |  /
     *     QUxAD
     *    /     \
     *   3       2
     *             +x
     */

    int m1speed = 0; int m2speed = 0; int m3speed = 0; int m4speed = 0;
    float m1f = 0; float m2f = 0; float m3f = 0; float m4f = 0;

    float xFactor, yFactor, zFactor, zPosFactor;

    xFactor = yFactor = 0.3; // units: (motor ticks) per (deg/s)
    zFactor = 0.3; // same units
    zPosFactor = 1.0;

	blue.println("adjustMotors with angle");

    float zcos = cos(zAngle *3.14159 / 180);
    float zsin = sin(zAngle *3.14159 / 180);

	// adjust for pitch and roll
	m1f += xAngCorrect * xFactor * zcos;
    m3f -= xAngCorrect * xFactor * zcos;
    m2f += xAngCorrect * xFactor * zsin;
    m4f -= xAngCorrect * xFactor * zsin;

	m4f += yAngCorrect * yFactor * zcos;
	m2f -= yAngCorrect * yFactor * zcos;
	m1f += yAngCorrect * yFactor * zsin;
	m3f -= yAngCorrect * yFactor * zsin;

	////adjust for altitude
	m1f += zPosCorrect * zFactor;
	m2f += zPosCorrect * zFactor;
	m3f += zPosCorrect * zFactor;
	m4f += zPosCorrect * zFactor;

	//// adjust for yaw

	m1f -= zAngCorrect;
	m3f -= zAngCorrect;

	m2f += zAngCorrect;
	m4f += zAngCorrect;

    // convert to motor outputs
    m1speed = static_cast<int>(m1f);
    m2speed = static_cast<int>(m2f);
    m3speed = static_cast<int>(m3f);
    m4speed = static_cast<int>(m4f);

	if(m1speed	>	MAX_MOTOR_SPEED)
		m1speed	=	MAX_MOTOR_SPEED;
	else if(m1speed < 0)
		m1speed = 0;
	if(m2speed	>	MAX_MOTOR_SPEED)
		m2speed	=	MAX_MOTOR_SPEED;
	else if(m2speed < 0)
		m2speed = 0;
	if(m3speed	>	MAX_MOTOR_SPEED)
		m3speed	=	MAX_MOTOR_SPEED;
	else if(m1speed < 0)
		m3speed = 0;
	if(m4speed	>	MAX_MOTOR_SPEED)
		m4speed	=	MAX_MOTOR_SPEED;
	else if(m4speed < 0)
		m4speed = 0;

	motorSet(MOTOR1, m1speed);
  	motorSet(MOTOR2, m2speed);
	motorSet(MOTOR3, m3speed);
  	motorSet(MOTOR4, m4speed);
}

int Quad::waitFor()
{
	int waitTime;
	blue.println("waitFor");

	/*waitTime = millis()-loopTime;
	Serial1.print(waitTime);
	blue.println("");*/

	if(waitTime < MAX_LOOP_TIME)
		delay(waitTime);
	loopTime=millis();
}

void Quad::readSerialCommand(void) {
	String serialData = blue.readLine();

	if(serialData == "")
		return;
	if(serialData == "x"){
		motorSet(MOTOR1, 0);
		motorSet(MOTOR2, 0);
		motorSet(MOTOR3, 20);
		motorSet(MOTOR4, 0);
		exit(1);
	}
	else if(serialData == "pu"){
		motorSet(MOTOR1, 0);
		motorSet(MOTOR2, 0);
		motorSet(MOTOR3, 20);
		motorSet(MOTOR4, 0);
		delay(500);
		xAngle.kP = xAngle.kP * 1.1;
		blue.print(xAngle.kP);
		blue.println("");
	}
	else if(serialData == "du"){
		motorSet(MOTOR1, 0);
		motorSet(MOTOR2, 0);
		motorSet(MOTOR3, 20);
		motorSet(MOTOR4, 0);
		delay(500);
		yAngle.kD = yAngle.kD * 1.1;
		blue.print(yAngle.kD);
		blue.println("");
	}
	else if(serialData == "iu"){
		motorSet(MOTOR1, 0);
		motorSet(MOTOR2, 0);
		motorSet(MOTOR3, 20);
		motorSet(MOTOR4, 0);
		delay(500);
		zAngle.kI = zAngle.kI * 1.1;
		blue.print(zAngle.kI);
		blue.println("");
	}
	else if(serialData == "pd"){
		motorSet(MOTOR1, 0);
		motorSet(MOTOR2, 0);
		motorSet(MOTOR3, 20);
		motorSet(MOTOR4, 0);
		delay(500);
		xAngle.kP = xAngle.kP / 1.1;
		blue.print(xAngle.kP);
		blue.println("");
	}
	else if(serialData == "dd"){
		motorSet(MOTOR1, 0);
		motorSet(MOTOR2, 0);
		motorSet(MOTOR3, 20);
		motorSet(MOTOR4, 0);
		delay(500);
		yAngle.kD = yAngle.kD / 1.1;
		blue.print(yAngle.kD);
		blue.println("");
	}
	else if(serialData == "id"){
		motorSet(MOTOR1, 0);
		motorSet(MOTOR2, 0);
		motorSet(MOTOR3, 20);
		motorSet(MOTOR4, 0);
		delay(500);
		zAngle.kI = zAngle.kI / 1.1;
		blue.print(zAngle.kI);
		blue.println("");
	}
	else{
		Serial.print("unknown command: ");
		Serial.println(serialData);
	}
}



