/************************************************/
/* Written by:  Aaron Derstine                  */
/* Date:        1/11/14                         */
/* Name:        SparkFun9DOF.h                  */
/* Description: This class represents the IMU   */
/*    for our quadrotor.  It contains the 3     */
/*    sensors and defines the 2 functions of    */
/*    the abstract IMU class                    */
/************************************************/

#ifndef _SPARKFUN9DOF_h
#define _SPARKFUN9DOF_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "IMU.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "HMC5883L.h"

class SparkFun9DOF : public IMU
{
public:
   SparkFun9DOF();
   ~SparkFun9DOF();

   // Initializes the IMU and the sensors it includes
   // Inputs:
   //    none
   // Returns:
   //    none
	virtual void initialize();

   // Gets the current data from the IMU
   // Inputs:
   //    data - IMUData struct that is modified to contain the current data from the IMU
   // Returns:
   //    RETURN_CODE indicating result of read operation
   virtual RETURN_CODE readData(struct IMUData &data);

   virtual ADXL345  *getAccelerometer() { return accelerometer; }
   virtual ITG3200  *getGyroscope()     { return gyroscope; }
   virtual HMC5883L *getMagnetometer()  { return magnetometer; }

private:
   // sensors in the IMU
   ADXL345 *accelerometer;
   ITG3200 *gyroscope;
   HMC5883L *magnetometer;
};

#endif

