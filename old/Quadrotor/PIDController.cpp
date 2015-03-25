// PIDController.cpp

#include "PIDController.h"

PIDController::PIDController()
{
   kP = 0.0;
   kI = 0.0;
   kD = 0.0;

   setpoint = 0.0;

   integralTermSum = 0.0;
   previousInput = 0.0;
}

void PIDController::initialize(float initialValue)
{
   previousInput = initialValue;
}

void PIDController::setConstants(float kP, float kI, float kD)
{
   setkP(kP);
   setkI(kI);
   setkD(kD);
}

void PIDController::setkP(float kP)
{
   this->kP = kP;
}

void PIDController::setkI(float kI)
{
   this->kI = kI;
}

void PIDController::setkD(float kD)
{
   this->kD = kD;
}

float PIDController::getkP() const 
{
   return kP;
}

float PIDController::getkI() const 
{
   return kI;
}

float PIDController::getkD() const 
{
   return kD;
}

void PIDController::setSetpoint(float setpoint)
{
   this->setpoint = setpoint;
}

float PIDController::getSetpoint() const
{
   return setpoint;
}

float PIDController::calculateOutput(float currentValue, float deltaTime)
{
   // basis for this code was found at http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
   // current error
   float error = setpoint - currentValue;

   // proportional term
   float pTerm = kP * error;

   // integral term
   integralTermSum += error * deltaTime;

   // derivative term
   float dTerm = kD * (currentValue - previousInput) / deltaTime;

   // update internal data
   previousInput = currentValue;

   return pTerm + kI * integralTermSum - dTerm;
}