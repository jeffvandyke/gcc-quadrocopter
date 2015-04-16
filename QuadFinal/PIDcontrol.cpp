#include "PIDcontrol.h"
using namespace std;

//starts the setpoint at bias
PIDController::PIDController(int p, int i, int d)
{
	setPoint=0;
	integralSumTerm=0;
	kp=p;
	kd=d;
	ki=i;
	previousMillis=millis();
}

PIDController::~PIDController()
{
}

void PIDController::move(int deltaPos)
{
  	setPoint+=deltaPos; 
}

void setSetPoint(int newPoint)
{
	setPoint=newPoint;
}

int PIDController::PD(int currentValue)
{
	// basis for this code was found at http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
	//Same as last year's code

	// update internal data
	previousInput = currentValue;
	int dTerm = kD * (currentValue - lastValue) / deltaTime

	//int pTerm - dTerm;
	return kP * (setpoint - currentValue) - dTerm;
}

int PIDController::PI(int currentValue)
{
	// basis for this code was found at http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
	//Same as last year's code
	// current error
	int error = setpoint - Value;

	// integral term
	integralTermSum += error * deltaTime;


	// update internal data
	previousInput = currentValue;

	//int pTerm + kI * integralTermSum ;
	return kP * error + kI * integralTermSum;
}

int PIDController::PID(int currentValue)
{
	// basis for this code was found at http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
	//Same as last year's code
	// current error
	error = setpoint - currentValue;
	// proportional term
	//int pTerm = kP * error;

	//deltaTime
	deltaTime=millis()-previousMillis;
	previousMillis=millis();

	// integral term
	integralTermSum += error * deltaTime;
	// derivative term
	dTerm = kD * (currentValue - previousInput) / deltaTime;

	// update internal data
	lastValue = currentValue;

	//int pTerm + kI * integralTermSum - dTerm;
	return kP * error + kI * integralTermSum - dTerm;
}

int PIDController::PID(int currentValue, int dTerm)
{
	// basis for this code was found at http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
	//Same as last year's code
	// current error
	int error = setpoint - currentValue;
	// proportional term
	//int pTerm = kP * error;

	//deltaTime
	deltaTime=millis()-previousMillis;
	previousMillis=millis();

	// integral term
	integralTermSum += error * deltaTime;

	// update internal data
	lastValue = currentValue;

	//int pTerm + kI * integralTermSum - dTerm;
	return kP * error + kI * integralTermSum - kD * dTerm;
}



