/************************************************/
/* Written by:  Aaron Derstine                  */
/* Date:        1/11/14                         */
/* Name:        ADXL345.h                       */
/* Description: This class represents the       */
/*    accelerometer in our IMU.  It provides    */
/*    an interface to retrieve the data from    */
/*    the sensor.                               */
/************************************************/

#ifndef _ADXL345_h
#define _ADXL345_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "QuadrotorAPI.h"
#include "AbstractIMUSensor.h"

// power mode
enum ADXL345_POWER_MODE
{
   LOW_POWER,
   NORMAL_POWER
};

// sample rate
enum ADXL345_BW_RATE
{
   RATE_3200HZ,
   RATE_1600HZ,
   RATE_800HZ,
   RATE_400HZ,
   RATE_200HZ,
   RATE_100HZ,
   RATE_50HZ,
   RATE_25HZ
};

// G range sensitivity
enum ADXL345_G_RANGE
{
   RANGE_2G,
   RANGE_4G,
   RANGE_8G,
   RANGE_16G
};

class ADXL345 : public AbstractIMUSensor
{
public:
   ADXL345();

   // Initializes the accelerometer with default settings and begins measurments
   // Inputs:
   //    none
   // Returns:
   //    none
   virtual void initialize();

   // Gets the current data from the accelerometer
   // Inputs:
   //    data - XYZData struct that is modified to contain the current data from the accelerometer
   // Returns:
   //    RETURN_CODE indicating result of read operation
   virtual RETURN_CODE readData(struct XYZData &data);

   // Indicates whether new data is available to be read from the accelerometer
   // Inputs:
   //    none
   // Returns:
   //    bool indicating whether new data is available to be read or not
   virtual bool dataAvailable();

   RETURN_CODE setSampleRate(ADXL345_POWER_MODE powerMode, ADXL345_BW_RATE sampleRate);
   RETURN_CODE setRange(ADXL345_G_RANGE range);
   RETURN_CODE enableMeasurements();

private:
   // scale to convert raw data into Gs
   float scaleFactor;
};

#endif

