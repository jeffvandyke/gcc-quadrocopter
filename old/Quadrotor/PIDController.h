/*******************************************/
/* Written by:  Aaron Derstine             */
/* Date:        1/18/14                    */
/* Name:        PIDController.h            */
/* Description: This class represents a    */
/*    controller used for stability and    */
/*    calculating the PID control loop.    */
/*******************************************/

#ifndef _PIDCONTROLLER_h
#define _PIDCONTROLLER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class PIDController
{
public:
   PIDController();

   void initialize(float initialValue);

   void setConstants(float kP, float kI, float kD);
   void setkP(float kP);
   void setkI(float kI);
   void setkD(float kD);

   float getkP() const;
   float getkI() const;
   float getkD() const;

   void setSetpoint(float setpoint);
   float getSetpoint() const;

   // deltaTime is seconds
   float calculateOutput(float currentValue, float deltaTime);

private:
   float kP;
   float kI;
   float kD;

   float setpoint;

   float integralTermSum; // used for the integral term
   float previousInput; // use the change in input for the derivative term to ignore changes in the setpoint
};

#endif

