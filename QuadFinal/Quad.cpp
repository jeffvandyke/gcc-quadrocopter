#include "Quad.h"
#include <Wire.h>
#include "I2C.h"
#include "Bluetooth.h"

#define MOTOR_OUT 6
#define MOTOR_TRIG 53
#define OUTPIN 6
#define TRIGPIN 53
#define PWMIN 100
#define PWHOVER 150

#define MAX_LOOP_TIME 25

#define MAX_MOTOR_SPEED 228
#define MAX_ANGULAR_DEFLECTION 20
#define MIN_ANGULAR_DEFLECTION -20
#define MAX_ALTITUDE_CORRECTION 100

#define MOTOR1 5
#define MOTOR2 2
#define MOTOR3 6
#define MOTOR4 3

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

#define X_ANG_KP 0.6 // prev was 0.5
#define X_ANG_KI 0.00003 // 30 // prev was 0.001, too high
#define X_ANG_KD 0.2

#define Y_ANG_KP 0.42
#define Y_ANG_KI 0
#define Y_ANG_KD 0.13

#define Z_ANG_KP 0
#define Z_ANG_KI 0
#define Z_ANG_KD 0

Quad::Quad(void)
{
	i2c = I2C();

	xPosition.changeGain(X_POS_KP, X_POS_KI, X_POS_KD);
	yPosition.changeGain(Y_POS_KP, Y_POS_KI, Y_POS_KD);
	zPosition.changeGain(Z_POS_KP, Z_POS_KI, Z_POS_KD);

	xAngle.changeGain(X_ANG_KP, X_ANG_KI, X_ANG_KD);
	yAngle.changeGain(Y_ANG_KP, Y_ANG_KI, Y_ANG_KD);
	zAngle.changeGain(Z_ANG_KP, Z_ANG_KI, Z_ANG_KD);

	return;
}

#if 1
void slog(String var, float val) {
    //Serial.print(var); Serial.print(":");
    Serial.print(val); Serial.print(",");
}

void slog(String var, int val) {
    //Serial.print(var); Serial.print(":");
    Serial.print(val); Serial.print(",");
}

void slog(float val) {
    Serial.print(val); Serial.print(",");
}

void slog(int val) {
    Serial.print(val); Serial.print(",");
}

void slogr() { Serial.print("\n"); }
#else
void slog(String var, float val) {
    Serial1.print(var); Serial1.print(",");
    Serial1.print(val); Serial1.print("\t");
}

void slog(String var, int val) {
    Serial1.print(var); Serial1.print(",");
    Serial1.print(val); Serial1.print("\t");
}

void slog(float val) {
    Serial1.print(val); Serial1.print("\t");
}

void slog(int val) {
    Serial1.print(val); Serial1.print("\t");
}

void slogr() { Serial1.print("\n"); }
#endif

Quad::~Quad(void)
{
}

int Quad::motorInitialize(void)
{
	pinMode(MOTOR1,OUTPUT);
	pinMode(MOTOR2,OUTPUT);
	pinMode(MOTOR3,OUTPUT);
	pinMode(MOTOR4,OUTPUT);
	delay(500);
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

	// analogWrite(motor, 0);
	analogWrite(motor, speed);

	return 0;
}

float prevXangle = 0;
float prevYangle = 0;

int Quad::setup(void)
{
	i2c.initialize();
	// Serial.begin(9600);
    // debugging over usb
	Serial.begin(115200);
	acc.setup();
	gyro.setup();
  	bar.setup();
	comp.setup();
	motorInitialize();
	blue.initialize();
    float barAvg = 0, gxAvg = 0,gyAvg = 0, gzAvg = 0;
    for(int i = 0; i < 30; i++) {
        barAvg += static_cast<float>(bar.readRawAltitude());
        gxAvg += static_cast<float>(gyro.readRawX());
        gyAvg += static_cast<float>(gyro.readRawY());
        gzAvg += static_cast<float>(gyro.readRawZ());
    }
    barAvg /= 30; gxAvg /= 30; gyAvg /= 30; gzAvg /= 30;
	//blue.println("setup");

	//gpssetup
	Serial2.begin(4800);
	delay(1000);

//	findSensorBias();

    // ordinarily, these would be the average of the gyro, acc, gps axis
    // over several samples, but these may be just fine for our purposes.
    Filter.initialize( 6.9158f, 9.9410f, 21.515f,
            gxAvg, gyAvg, gzAvg,
            //gyro -16.027f, 0.9157f, 0.6185f,
            //         -12504.f, -13316.f, -24942.f );
        // gps = bogus values, so for now, bias is zero
        0,0,barAvg);

	loopTime=millis();
	quadState = Filter.getQuadState();
	return 0;
}

// global counter, UNTESTED
int nIteration = 0;

int Quad::executeCycle(void)
{

    nIteration++;
    //slog(nIteration);

	//this->readSerialCommand();
	if(blue.readLine() == "x")
		exit(1);
	//blue.println("!!executeCycle!!");

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


    int kTime = millis();
	Filter.assignSensorValues(
		accX, accY, accZ,	// acceleration
		gyroX, gyroY, gyroZ,	// gyroscope
		 compX, compY, compZ,	// Compass
		 0, 0, barZ,
         nIteration % 10 == 0);

    // slog("k-w", (int)millis() - kTime);
    kTime = millis();

	Filter.predictAndUpdate();

	// blue.println("done");

	//retrieve values from Kalman Filter
	quadState = Filter.getQuadState();
    //slog("k-W", (int)millis() - kTime);

    slog("stXang", quadState.xAngle);
	slog("stYang", quadState.yAngle);
    slog("stZang", quadState.zAngle);
    //slog("stzpos", quadState.zPosition);

	// blue.println("Angles:");
	// Serial1.print(quadState.xAngle);
	// blue.println("");
	//Serial1.print(quadState.xRotation);
	//blue.println(" :xRotation");
	// Serial1.print(quadState.yAngle);
	// blue.println("");
	//Serial1.print(quadState.yRotation);
	//blue.println(" :yRotation");
	// Serial1.print(quadState.zAngle);
	// blue.println("");
	//Serial1.print(quadState.yRotation);
	//blue.println(" :zRotation");

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




    xAngCorrect	= xAngle.PID(quadState.xAngle);
	yAngCorrect = yAngle.PID(quadState.yAngle);
	zAngCorrect = zAngle.PID(quadState.zAngle);

    slog("xC", xAngCorrect);
	writeFloat(1,0.003f);
    //slog("yC", yAngCorrect);
    //slog("zC", zAngCorrect);
	//slogr();
    // only adjust y for now
    yAngCorrect = 0;
    zAngCorrect = 0;

	//adjustMotors();
	adjustMotors(quadState.zAngle);


	//wait until a constant time has passed.
	waitFor();

    slogr();

	return 0;
}

void Quad::writeFloat(byte id, float f)
{
	byte * b = (byte *) &f;
	Serial1.write(id);
	Serial1.write(b,4);
}

//updates all of the sensor values stored in the Quad object
int Quad::getSensorVals(void)
{
//	blue.println("getSensorVals");

    int sTime = millis();
	//Accelerometer
	accX=	acc.readRawX();
	accY=	acc.readRawY();
	accZ=	acc.readRawZ();
	//Gyroscope
	gyroX=	gyro.readRawX();
	gyroY=	gyro.readRawY();
	gyroZ=	gyro.readRawZ();
	//Compass
	compX=	comp.getRawX();
	compY=	comp.getRawY();
	compZ=	comp.getRawZ();
    if(nIteration % 10 == 0)
        barZ = bar.readRawAltitude();
    // slog("barZ", barZ);

    // slog("s-W", (int)millis() - sTime);

}

//updates the internal GPS values
//int Quad::getGPSval(void)
//{
//	//waits until a full signal is read from the gps
//	while (Serial2.available())
//     {
//		char c = Serial2.read();
//		if(gps.encode(c))
//			break;
//	}
//
//	//since there is no way to tell how stale the value that the GPS holds is, this function merely checks to see if
//	//its a new value or not
//	if (gps.readRawLat() == GPSlat)
//		return false;
//
//	GPSlat	=	gps.readRawLat()/*-gpsLatBias*/;
//	GPSlong	=	gps.readRawLong()/*-gpsLongBias*/;
//	GPSalt	=	gps.readRawAlt()/*-gpsAltBias*/;
//	return true;
//}

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


// motor correction that takes z angle into account
int Quad::adjustMotors(int zAngle)
{
    // vetted: good! as long as zAngle = 0 corresponds to N || usb
    zAngle = zAngle - 45; // for matching this coordinate system
    // slog("adjm:", (float)zAngle);
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

    xFactor = yFactor = 1; // units: (motor ticks) per (deg/s)
    zFactor = 1; // same units
    zPosFactor = 1.0;

	//blue.println("adjustMotors with angle");

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


    //slog("m1",m1speed);
    //slog("m2",m2speed);
    //slog("m3",m3speed);
    //slog("m4",m4speed);

    //m2speed = -m2speed;
    //m4speed = -m4speed;

    m1speed += PWHOVER;
    m2speed += PWHOVER;
    m3speed += PWHOVER;
    m4speed += PWHOVER;

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


    slog("M1", m1speed);
    slog("M2", m2speed);
    slog("M3", m3speed);
    slog("M4", m4speed);


	m1speed = 0;
	m3speed = 0;

    //if(nIteration % 0 == 0) {
        motorSet(MOTOR1, m1speed);
        motorSet(MOTOR2, m2speed);
        motorSet(MOTOR3, m3speed);
        motorSet(MOTOR4, m4speed);
    //}
}

int Quad::waitFor()
{
	int waitTime;
	//blue.println("waitFor");

	waitTime = millis()-loopTime;
    slog("wT", waitTime);
	// Serial1.print(waitTime);
	// blue.println("");

	if(waitTime < MAX_LOOP_TIME)
		delay(MAX_LOOP_TIME - waitTime);
	loopTime=millis();
}

void Quad::readSerialCommand(void) {
	//String serialData = blue.readLine();

	//if(serialData == "")
	//	return;
	//if(serialData == "x"){
	//	motorSet(MOTOR1, 0);
	//	motorSet(MOTOR2, 0);
	//	motorSet(MOTOR3, 20);
	//	motorSet(MOTOR4, 0);
	//	exit(1);
	//}
	//else if(serialData == "pu"){
	//	motorSet(MOTOR1, 0);
	//	motorSet(MOTOR2, 0);
	//	motorSet(MOTOR3, 20);
	//	motorSet(MOTOR4, 0);
	//	delay(500);
	//	xAngle.kP = xAngle.kP * 1.1;
	//	blue.print(xAngle.kP);
	//	blue.println("");
	//}
	//else if(serialData == "du"){
	//	motorSet(MOTOR1, 0);
	//	motorSet(MOTOR2, 0);
	//	motorSet(MOTOR3, 20);
	//	motorSet(MOTOR4, 0);
	//	delay(500);
	//	yAngle.kD = yAngle.kD * 1.1;
	//	blue.print(yAngle.kD);
	//	blue.println("");
	//}
	//else if(serialData == "iu"){
	//	motorSet(MOTOR1, 0);
	//	motorSet(MOTOR2, 0);
	//	motorSet(MOTOR3, 20);
	//	motorSet(MOTOR4, 0);
	//	delay(500);
	//	zAngle.kI = zAngle.kI * 1.1;
	//	blue.print(zAngle.kI);
	//	blue.println("");
	//}
	//else if(serialData == "pd"){
	//	motorSet(MOTOR1, 0);
	//	motorSet(MOTOR2, 0);
	//	motorSet(MOTOR3, 20);
	//	motorSet(MOTOR4, 0);
	//	delay(500);
	//	xAngle.kP = xAngle.kP / 1.1;
	//	blue.print(xAngle.kP);
	//	blue.println("");
	//}
	//else if(serialData == "dd"){
	//	motorSet(MOTOR1, 0);
	//	motorSet(MOTOR2, 0);
	//	motorSet(MOTOR3, 20);
	//	motorSet(MOTOR4, 0);
	//	delay(500);
	//	yAngle.kD = yAngle.kD / 1.1;
	//	blue.print(yAngle.kD);
	//	blue.println("");
	//}
	//else if(serialData == "id"){
	//	motorSet(MOTOR1, 0);
	//	motorSet(MOTOR2, 0);
	//	motorSet(MOTOR3, 20);
	//	motorSet(MOTOR4, 0);
	//	delay(500);
	//	zAngle.kI = zAngle.kI / 1.1;
	//	blue.print(zAngle.kI);
	//	blue.println("");
	//}
	//else{
	//	Serial.print("unknown command: ");
	//	Serial.println(serialData);
	//}
}



