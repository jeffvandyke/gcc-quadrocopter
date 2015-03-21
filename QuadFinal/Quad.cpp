#include "Quad.h"
#include <Wire.h>
#include <I2C.h>
#include "Bluetooth.h"

#define MOTOR_OUT 6
#define MOTOR_TRIG 53
#define OUTPIN 6
#define TRIGPIN 53

Quad::Quad(void)
{
	i2c = I2C();
	return;
}

Quad::~Quad(void)
{
}

int motorInitialize()
{
	pinMode(OUTPIN,OUTPUT);
	pinMode(TRIGPIN,OUTPUT);
	digitalWrite(TRIGPIN,LOW);
	analogWrite(OUTPIN,0);
	delay(3000);
	analogWrite(OUTPIN,100);
	delay(1000);
	analogWrite(OUTPIN,0);
	delay(200);

	return 0;
}

int motorSet(int dutyCycle)
{
	digitalWrite(TRIGPIN,HIGH);
	analogWrite(OUTPIN,dutyCycle);
	digitalWrite(TRIGPIN,LOW);

	Serial.print("Duty cycle set to ");
	Serial.print(dutyCycle);
	Serial.print("\n");

	return 0;
}

int Quad::setup(void)
{
	sensorMode = ALL;
	i2c.initialize();
	Serial.begin(9600);
	Serial.print("IMUTester::setupTester called");
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

	int sensorData;

	while (Serial2.available())
     {
		char c = Serial2.read();
		if(gps.encode(c))
			break;
	}

	/*Serial.print("Accelerometer: \n");		
	Serial.print(acc.readRawX());           Serial.print(" , ");
	Serial.print(acc.readRawY());           Serial.print(" , ");
	Serial.print(acc.readRawZ());           Serial.print("\n");
	Serial.print("Gyroscope: \n");
	Serial.print(gyro.readRawX());          Serial.print(" , ");
	Serial.print(gyro.readRawY());          Serial.print(" , ");
	Serial.print(gyro.readRawZ());          Serial.print("\n");
	Serial.print("Compass: \n");
	Serial.print(comp.getRawX());           Serial.print(" , ");
	Serial.print(comp.getRawY());           Serial.print(" , ");
	Serial.print(comp.getRawZ());			Serial.print("\n");*/
	Serial.print("GPS: \n");
	Serial.print(gps.readRawLat());			Serial.print(" , ");
	Serial.print(gps.readRawLong());		Serial.print("\n");

	delay(10);
	return 0;
}

void Quad::readSerialCommand() {
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