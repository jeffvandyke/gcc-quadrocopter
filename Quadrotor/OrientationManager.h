/************************************************/
/* Written by:  Aaron Derstine                  */
/* Date:        1/11/14                         */
/* Name:        OrientationManager.h            */
/* Description: This class manages the IMU and  */
/*    all calculations in determining the roll, */
/*    pitch, and yaw of the quadrotor in flight */
/************************************************/

#ifndef _ORIENTATIONMANAGER_h
#define _ORIENTATIONMANAGER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "QuadrotorAPI.h"
#include "IMU.h"
#include "Ping.h"

class OrientationManager
{
public:
   OrientationManager();
   RETURN_CODE initialize(IMU *imu, Ping *altitudeSensor);

   // IMPORTANT!
   // call this function at the end of the initialization sequence for the quadrotor before entering the main loop
   // it will use the accelerometer data as the basis for the gyro integrations
   // it also begins to keep track of the delta time for the integration
   RETURN_CODE initialRead(struct RPYData &orientation);

   RETURN_CODE setGyroWeight(float weight);

   RETURN_CODE getCurrentOrientation(float deltaTime, struct RPYData &orientation);
   RETURN_CODE getCurrentOrientation(float deltaTime, struct RPYData &filteredOrientation, struct RPYData &accel, struct RPYData &gyro, float &magYaw);

   // the following functions are used by getCurrentOrientation() so will not need to be called externally
   float calculateAccelerometerRoll(const struct XYZData &accelData);
   float calculateAccelerometerPitch(const struct XYZData &accelData);

   float calculateGyroscopeRoll(const struct XYZData &gyroData, float previousGyroRoll, float dt);
   float calculateGyroscopePitch(const struct XYZData &gyroData, float previousGyroPitch, float dt);
   float calculateGyroscopeYaw(const struct XYZData &gyroData, float previousGyroYaw, float dt);

   float calculateMagnetometerYaw(const struct XYZData &magData, float roll, float pitch);
   float calculateMagnetometerYawNoTiltCompensation(const struct XYZData &magData);

   float calculateFilteredRoll(float accelRoll, float gyroRoll);
   float calculateFilteredPitch(float accelPitch, float gyroPitch);
   float calculateFilteredYaw(float magYaw, float gyroYaw);



private:
   RETURN_CODE readIMU();

   IMU  *imu;
   Ping *altitudeSensor;
   struct IMUData imuData;
   struct RPYData previousGyroData;
   float gyroWeight;
};

#endif

