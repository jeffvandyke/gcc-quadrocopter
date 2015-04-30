#include "PIDcontrol.h"
#define P_DELTAT 0.025
using namespace std;

PIDController::PIDController()
{
	setPoint=0;
	integralSumTerm=0;
	kP=0;
	kD=0;
	kI=0;
	previousMillis=millis();
    deltaTime = P_DELTAT;
}

//starts the setpoint at bias
void PIDController::changeGain(float p, float i, float d)
{
	kP=p;
	kD=d;
	kI=i;
}

PIDController::~PIDController()
{
}

void PIDController::move(int deltaPos)
{
  	setPoint+=deltaPos; 
}

void PIDController::setSetPoint(float newPoint)
{
	setPoint=newPoint;
}

float PIDController::PID(float currentValue)
{
	// basis for this code was found at http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
	//Same as last year's code
	// current error
	error = setPoint - currentValue;
	// proportional term
	//int pTerm = kP * error;

	//deltaTime
	// deltaTime=millis()-previousMillis;
	previousMillis=millis();

	// integral term
	integralSumTerm += error * deltaTime;
	// derivative term
	dTerm = dTerm * 0.0 + 1.0 * (kD * (currentValue - lastValue) / deltaTime);

	// update internal data
	lastValue = currentValue;

	//int pTerm + kI * integralTermSum - dTerm;
	return kP * error + kI * integralSumTerm - dTerm;
}

float PIDController::PID(float currentValue, float dTerm)
{
	// basis for this code was found at http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
	//Same as last year's code
	// current error
	error = setPoint - currentValue;
	// proportional term
	//int pTerm = kP * error;

	//deltaTime
	deltaTime=millis()-previousMillis;
	previousMillis=millis();

	// integral term
	integralSumTerm += error * deltaTime;

	// Serial1.print(integralSumTerm);
	// Serial1.println();

	// update internal data
	lastValue = currentValue;

	//int pTerm + kI * integralTermSum - dTerm;
	return kP * error + kI * integralSumTerm - kD * dTerm;
}



