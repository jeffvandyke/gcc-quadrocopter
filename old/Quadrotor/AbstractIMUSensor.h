/************************************************/
/* Written by:  Aaron Derstine                  */
/* Date:        1/17/14                         */
/* Name:        AbstractIMUSensor.h             */
/* Description: This class represents an        */
/*    abstract sensor that would be found on an */
/*    IMU.  It declares the functions that must */
/*    be defined in an IMU sensor.              */
/************************************************/

#ifndef _ABSTRACTIMUSENSOR_h
#define _ABSTRACTIMUSENSOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "QuadrotorAPI.h"

class AbstractIMUSensor
{
public:
   // Initializes the IMU sensor
   // Notes:
   //    Needs to be defined in a derived class
   // Inputs:
   //    none
   // Returns:
   //    none
   virtual void initialize() = 0;

   // Gets the current data from the IMU sensor
   // Notes:
   //    Needs to be defined in a derived class
   // Inputs:
   //    data - XYZData struct that is modified to contain the current data from the IMU sensor
   // Returns:
   //    RETURN_CODE indicating result of read operation
   virtual RETURN_CODE readData(struct XYZData &data) = 0;

   // Indicates whether new data is available to be read from the IMU sensor
   // Notes:
   //    should be overridden in derived class if possible but by default simply returns true
   // Inputs:
   //    none
   // Returns:
   //    bool indicating whether new data is available to be read or not
   virtual bool dataAvailable() { return true; }

private:

};

#endif

