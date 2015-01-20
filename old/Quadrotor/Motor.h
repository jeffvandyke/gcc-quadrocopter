/************************************************/
/* Written by:  Tom Kuczun                      */
/* Date:        1/11/14                         */
/* Name:        Motor.h                         */
/* Description: This class represents a motor   */
/*    used on the quadrotor.  There will be 4   */
/*    instances of this class.  It communicates */
/*    with the Arduino via analogWrite().       */
/************************************************/

#ifndef _MOTOR_h
#define _MOTOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "QuadrotorAPI.h"

enum Spin		
{
   CLOCKWISE, 
   COUNTER_CLOCKWISE
};

enum MotorID
{
   UNSET, 
   ONE, 
   TWO, 
   THREE, 
   FOUR
};

class Motor
{
public:
   Motor();

   // Initializes the motor and turns it on
   // Notes:
   //    it is important that the spin and id are correct so the control system controls the motor correctly
   // Inputs:
   //    spin - a Spin indicating the direction to turn the motor
   //    id   - a MotorID indicating which of the 4 motors it represents
   // Returns:
   //    none
   void initialize(Spin spin, MotorID id);

   // between 0 and 100
   RETURN_CODE setSpeed(float value);
   float getSpeed() const;
   MotorID getId() const;
   Spin getSpin() const;

private:	
   // keeps track of the motor settings
   Spin    spin; // direction
   MotorID id;
   float   speed;  
   int     pin; // pin on the Arduino the ESC is connected to
};
#endif
