/************************************************/
/* Written by:  Aaron Derstine                  */
/* Date:        1/11/14                         */
/* Name:        Quadrotor.h                     */
/* Description: This class represents the       */
/*    quadrotor.  It only has the two functions */
/*    that are called automatically by the      */
/*    main Arduino routine.                     */
/************************************************/

#ifndef _QUADROTOR_h
#define _QUADROTOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "QuadrotorAPI.h"
#include "Bluetooth.h"
#include "IMU.h"
#include "Motor.h"
#include "Ping.h"
#include "OrientationManager.h"
#include "ControllerManager.h"
#include "TestCompass.h"

class Quadrotor
{
public:
   Quadrotor();
   ~Quadrotor();

   // Initializes the quadrotor and its components
   // Notes:
   //    Will be called automatically by the main Arduino routine before the loop function is called infinitely
   // Inputs:
   //    none
   // Returns:
   //    none
   void initialize();

   // Main loop routine for the quadrotor.  All control and flight sequences get called here.
   // Notes:
   //    Will be called automatically by the main Arduino routine in an infinite loop
   // Inputs:
   //    none
   // Returns:
   //    none
   void loop();

   void handleMessage(String message);
   // for landing smoothly
   // hasn't been too extensively testing but works okay
   void shutdownSequence();
   // startingSpeed is the initial speed to rotate the motors
   // should be low enough that it doesn't try to take off
   void startupSequence(float startingSpeed);
   void waitForStartCommand();

private:
   // quadrotor components
   IMU *imu;
   Motor *motor1, *motor2, *motor3, *motor4;
   OrientationManager *orientationManager;
   ControllerManager  *controller;
   Bluetooth          *bluetooth;
   Ping				  *altitudeSensor;

   // struct to keep track of the current orientation in flight (roll, pitch, yaw)
   struct RPYData orientation;


   String message;
   unsigned long previousTime;
   unsigned long currentTime;
   float deltaTime;
};

#endif
