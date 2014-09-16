/************************************************/
/* Written by:  Aaron Derstine                  */
/* Date:        1/18/14                         */
/* Name:        ControllerManager.h             */
/* Description: This class represents the       */
/*    overall control system for the quadrotor. */
/*    It has a PID controller for roll, pitch,  */
/*    yaw, and altitude and controls the motors */
/*    to keep it in stable flight.              */
/************************************************/

#ifndef _CONTROLLERMANAGER_h
#define _CONTROLLERMANAGER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "QuadrotorAPI.h"
#include "PIDController.h"
#include "Motor.h"

struct ControllerOutput
{
   float pitch;
   float roll;
   float yaw;
   float altitude;
};

class ControllerManager
{
public:
   ControllerManager();
   ~ControllerManager();

   void initialize(const struct RPYData &initialData, float startingSpeed);
   // so it can control the motors
   void attachMotors(Motor *motor1, Motor *motor2, Motor *motor3, Motor *motor4);

   void flightControl(const struct RPYData &currentOrientation, float deltaTime);
   // output is for debugging purposes
   void flightControl(const struct RPYData &currentOrientation, float deltaTime, struct ControllerOutput &output);

   PIDController *getPitchController()    { return pitchController; }
   PIDController *getRollController()     { return rollController; }
   PIDController *getYawController()      { return yawController; }
   PIDController *getAltitudeController() { return altitudeController; }

private:
   void adjustMotorValues(float rollCorrection, float pitchCorrection, float yawCorrection, float altitudeCorrection);

   Motor *motor1, *motor2, *motor3, *motor4;

   PIDController *rollController;
   PIDController *pitchController;
   PIDController *yawController;
   PIDController *altitudeController;

   float currentMotorMax;
};

#endif

