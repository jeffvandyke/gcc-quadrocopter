/************************************************/
/* Written by:  Aaron Derstine                  */
/* Date:        1/11/14                         */
/* Name:        IMU.h                           */
/* Description: This class represents an        */
/*    abstract IMU with two functions that must */
/*    be provided by any IMU.  The definition   */
/*    of a struct that holds data from a IMU    */
/*    is also defined here                      */
/************************************************/

#ifndef _IMU_h
#define _IMU_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "QuadrotorAPI.h"
#include "AbstractIMUSensor.h"

// stores all the raw data from an IMU
struct IMUData
{
   struct XYZData accelerometer;
   struct XYZData gyroscope;
   struct XYZData magnetometer;
};

class IMU
{
public:
   // Initializes the IMU and the sensors it includes
   // Notes:
   //    Needs to be defined in a derived class
   // Inputs:
   //    none
   // Returns:
   //    none
   virtual void initialize() = 0;

   // Gets the current data from the IMU
   // Notes:
   //    Needs to be defined in a derived class
   // Inputs:
   //    data - IMUData struct that is modified to contain the current data from the IMU
   // Returns:
   //    RETURN_CODE indicating result of read operation
   virtual RETURN_CODE readData(struct IMUData &data) = 0;

   virtual AbstractIMUSensor *getAccelerometer() = 0;
   virtual AbstractIMUSensor *getGyroscope() = 0;
   virtual AbstractIMUSensor *getMagnetometer() = 0;

private:
   // need concrete sensors in derived class
   // accelerometer;
   // gyroscope;
   // magnetometer;
};

#endif

