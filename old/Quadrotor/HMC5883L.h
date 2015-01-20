/************************************************/
/* Written by:  Aaron Derstine                  */
/* Date:        1/11/14                         */
/* Name:        HMC5883L.h                      */
/* Description: This class represents the       */
/*    magnetometer in our IMU.  It provides     */
/*    an interface to retrieve the data from    */
/*    the sensor.                               */
/************************************************/

#ifndef _HMC5883L_h
#define _HMC5883L_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "QuadrotorAPI.h"
#include "AbstractIMUSensor.h"

// gauss field range settings
enum HMC5883L_SENSOR_FIELD_RANGE
{
   GAUSS_88,
   GAUSS_1_3,
   GAUSS_1_9,
   GAUSS_2_5,
   GAUSS_4_0,
   GAUSS_4_7,
   GAUSS_5_6,
   GAUSS_8_1
};

class HMC5883L : public AbstractIMUSensor
{
public:
   HMC5883L();

   // Initializes the magnetometer with default settings and begins measurments
   // Inputs:
   //    none
   // Returns:
   //    none
   virtual void initialize();

   // Gets the current data from the magnetometer
   // Inputs:
   //    data - XYZData struct that is modified to contain the current data from the magnetometer
   // Returns:
   //    RETURN_CODE indicating result of read operation
   virtual RETURN_CODE readData(struct XYZData &data);

   // Indicates whether new data is available to be read from the magnetometer
   // Inputs:
   //    none
   // Returns:
   //    bool indicating whether new data is available to be read or not
   virtual bool dataAvailable();

   RETURN_CODE setRange(HMC5883L_SENSOR_FIELD_RANGE gauss);


   // runs the HMC5883L self test
   void selfTest();

private:
   // scaleFactor depends on the range settings
   float scaleFactor;
};

#endif