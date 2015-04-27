#include <Wire.h>
//#include "PIDControl.h"

#define MOTOR_OUT 6
#define MOTOR_TRIG 53
#define OUTPIN 6
#define TRIGPIN 53
#define PWMIN 100

#define MAX_MOTOR_SPEED 128
#define MAX_ANGULAR_DEFLECTION 20
#define MIN_ANGULAR_DEFLECTION -20
#define MAX_ALTITUDE_CORRECTION 100

#define MOTOR1 6
#define MOTOR2 3
#define MOTOR3 5
#define MOTOR4 2

int i=0;

void setup()
{
	pinMode(MOTOR1,OUTPUT);
	pinMode(MOTOR2,OUTPUT);
	pinMode(MOTOR3,OUTPUT);
	pinMode(MOTOR4,OUTPUT);

	motorInitialize();
}

void loop()
{
	motorSet(MOTOR1, i);
	delay(500);
	i++;
}

int motorInitialize(void)
{
	analogWrite(2,0);
	analogWrite(3,0);
	analogWrite(5,0);
	analogWrite(6,0);
	delay(2000);
	analogWrite(2,PWMIN);
	analogWrite(3,PWMIN);
	analogWrite(5,PWMIN);
	analogWrite(6,PWMIN);
	delay(2000);
	analogWrite(2,0);
	analogWrite(3,0);
	analogWrite(5,0);
	analogWrite(6,0);
	delay(2000);

	return 0;
}

int motorSet(int motor, int speed)
{
	speed	=	speed + PWMIN;

	analogWrite(motor, speed);

//	delay(1000);
//	analogWrite(motor, 0);
//	delay(1000);


	return 0;
}