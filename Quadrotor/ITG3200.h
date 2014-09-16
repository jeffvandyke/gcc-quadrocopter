/************************************************/
/* Written by:  Aaron Derstine                  */
/* Date:        1/11/14                         */
/* Name:        ITG3200.h                       */
/* Description: This class represents the       */
/*    gyroscope in our IMU.  It provides        */
/*    an interface to retrieve the data from    */
/*    the sensor.                               */
/************************************************/

#ifndef _ITG3200_h
#define _ITG3200_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "QuadrotorAPI.h"
#include "AbstractIMUSensor.h"

// digital low-pass filter configuration bandwidth options
enum ITG3200_DLPF_CONFIG
{
   BW_256HZ,
   BW_188HZ,
   BW_98HZ,
   BW_42HZ,
   BW_20HZ,
   BW_10HZ,
   BW_5HZ
};

class ITG3200 : public AbstractIMUSensor
{
public:
   // Initializes the gyroscope with default settings and begins measurments
   // Inputs:
   //    none
   // Returns:
   //    none
   virtual void initialize();

   // Gets the current data from the gyroscope
   // Inputs:
   //    data - XYZData struct that is modified to contain the current data from the gyroscope
   // Returns:
   //    RETURN_CODE indicating result of read operation
   virtual RETURN_CODE readData(struct XYZData &data);

   // Indicates whether new data is available to be read from the gyroscope
   // Inputs:
   //    none
   // Returns:
   //    bool indicating whether new data is available to be read or not
   virtual bool dataAvailable();

   // Gets the current temperature in Farenheit from the gyroscope's temperature sensor
   // Inputs:
   //    temp - float that is modified to contain the current temperature data from the gyroscope
   // Returns:
   //    RETURN_CODE indicating result of read operation
   RETURN_CODE readTemperature(float &temp);

 

   // between 0 and 255
   RETURN_CODE setSampleRateDivider(byte divider);
   // note: will also affect the internal sampling rate - see data sheet
   RETURN_CODE setDLPFConfiguration(ITG3200_DLPF_CONFIG bw);

private:
   
};

#endif