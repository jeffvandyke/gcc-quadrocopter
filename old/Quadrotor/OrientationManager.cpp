// OrientationManager.cpp

#include "OrientationManager.h"

#define PITCH_OFFSET 0.0  // need to offset the pitch by -0.3 deg because of our orientation on the quadrotor

OrientationManager::OrientationManager()
{
   imu = NULL;
   altitudeSensor = NULL;

   // what percentage of the gyro data compared to the accelerometer or magnetometer
   setGyroWeight(0.98);
}

RETURN_CODE OrientationManager::initialize(IMU *imu, Ping *altitudeSensor)
{
   // initialize the imu
   this->imu = imu;
   this->imu->initialize();

   // read imu so that the initialization settings take effect
   if (readIMU() != SUCCESS)
   {
      // some error occured
      Serial.println("Error reading IMU in OrientationManager::initialize()");
      return ERROR;
   }

   // connect altitude sensor
   this->altitudeSensor = altitudeSensor;

   return SUCCESS;
}

RETURN_CODE OrientationManager::initialRead(struct RPYData &orientation)
{
   // read imu and use the results of the accelerometer and magnetometer as the basis for the gyro
   if (readIMU() != SUCCESS)
   {
      // some error occured
      Serial.println("Error reading IMU in OrientationManager::initialRead()");
      return ERROR;
   }  

   // save the accelerometer readings as the initial gyro readings for the integration to start from
   previousGyroData.pitch = calculateAccelerometerPitch(imuData.accelerometer);
   previousGyroData.roll  = calculateAccelerometerRoll(imuData.accelerometer);
   // just going to use gyro since mag doesn't work reliably
   //previousGyroData.yaw   = calculateMagnetometerYaw(imuData.magnetometer, previousGyroData.roll, previousGyroData.pitch); 
   previousGyroData.yaw = 180.0;
   
   // bit of a hack since not actually from they gyro but it is a free place to keep track of this
   previousGyroData.altitude = altitudeSensor->getDistanceInches(); 

   // update the struct to pass back data
   orientation.roll     = previousGyroData.roll;
   orientation.pitch    = previousGyroData.pitch;
   orientation.yaw      = previousGyroData.yaw; 
   orientation.altitude = previousGyroData.altitude;

   return SUCCESS;
}

RETURN_CODE OrientationManager::readIMU()
{
   RETURN_CODE retVal;
   // read imu to update internal data
   retVal = imu->readData(imuData);
   if (retVal != SUCCESS)
   {
      // some error occured
      sendErrorMessage(retVal, "OrientationManager::readIMU()");
      return ERROR;
   }  

   return SUCCESS;
}

RETURN_CODE OrientationManager::setGyroWeight(float weight)
{
   // make sure it is a reasonable selection
   if (weight <= 1.0 && weight >= 0.0)
   {
      gyroWeight = weight;
      return SUCCESS;
   }
   return INVALID;
}

// TODO: check and compare all calculations
float OrientationManager::calculateAccelerometerRoll(const struct XYZData &accelData)
{
   // subtract pi and negate so that it is 0 at level for our configuration
   float roll = -(atan2(accelData.X, accelData.Z) - PI);

   // if roll > PI make negative
   if (roll > PI)
      roll = -(TWO_PI - roll);

   return roll * RAD_TO_DEG;
}

float OrientationManager::calculateAccelerometerPitch(const struct XYZData &accelData)
{
   // subtract pi and negate so that it is 0 at level for our configuration
   float pitch = -(atan2(accelData.Y, accelData.Z) - PI);
   
   // if pitch > PI make negative
   if (pitch > PI)
      pitch = -(TWO_PI - pitch);

   return pitch * RAD_TO_DEG + PITCH_OFFSET;
}

// these constants that are either added or subtracted are an effort to reduce gyroscope "drift"
// they should probably be verified that they are accurate again
float OrientationManager::calculateGyroscopeRoll(const struct XYZData &gyroData, float previousGyroRoll, float dt)
{
   // Integrate the gyroscope data to find the angle
   return previousGyroRoll + (gyroData.Y * dt) + 0.0021;
}

float OrientationManager::calculateGyroscopePitch(const struct XYZData &gyroData, float previousGyroPitch, float dt)
{
   // Integrate the gyroscope data to find the angle
   return previousGyroPitch - (gyroData.X * dt) - 0.003;
}

float OrientationManager::calculateGyroscopeYaw(const struct XYZData &gyroData, float previousGyroYaw, float dt)
{
   // Integrate the gyroscope data to find the angle
   float yaw = previousGyroYaw - (gyroData.Z * dt) + 0.002;
   if      (yaw < 0)   yaw += 360;
   else if (yaw > 360) yaw -= 360;
   return yaw;
}

float OrientationManager::calculateMagnetometerYaw(const struct XYZData &magData, float roll, float pitch)
{
// convert from degrees to radians
   pitch *= DEG_TO_RAD;
   roll  *= DEG_TO_RAD;

   // perform tilt compensations
   // equations found at: http://www.loveelectronics.co.uk/Tutorials/13/tilt-compensated-compass-arduino-tutorial
   float headingX = -magData.X * cos(pitch) + magData.Z * sin(pitch);
   float headingY = -magData.X * sin(roll) * sin(pitch) + -magData.Y * cos(roll) - magData.Z * sin(roll) * cos(pitch);

   // calculate heading from raw data
   float heading = atan2(headingY, headingX);

   // magnetic declination of Grove City -9deg 23' = -.16377 radians
   // compensates for mag north not being true north
   float declinationAngle = -.16377;
   heading += declinationAngle;

   // Correct for when signs are reversed.
   if (heading < 0) heading += TWO_PI;
  
   // Check for wrap due to addition of declination.
   if (heading > TWO_PI) heading -= TWO_PI;
 
   // Convert radians to degrees for readability.
   float headingDegrees = heading * RAD_TO_DEG;

   return headingDegrees;
}

float OrientationManager::calculateFilteredRoll(float accelRoll, float gyroRoll)
{
   float roll = (gyroRoll * gyroWeight) + (accelRoll * (1-gyroWeight) );
   previousGyroData.roll = roll;
   return roll;
}

float OrientationManager::calculateFilteredPitch(float accelPitch, float gyroPitch)
{
   float pitch = (gyroPitch * gyroWeight) + (accelPitch * (1-gyroWeight) );
   previousGyroData.pitch = pitch;
   return pitch;
}

float OrientationManager::calculateFilteredYaw(float magYaw, float gyroYaw)
{
   float yaw = (gyroYaw); // * gyroWeight) + (magYaw * (1-gyroWeight) ); // We decided to not use the compass reading since it was not accurate
   previousGyroData.yaw = yaw;
   return yaw;
}

RETURN_CODE OrientationManager::getCurrentOrientation(float deltaTime, struct RPYData &orientation)
{
   // read imu to get the latest data
   if (readIMU() != SUCCESS)
   {
      // some error occured
      Serial1.println("Error reading IMU in OrientationManager::getCurrentOrientation()");
      return ERROR;
   }

   // calculate the roll and pitch angles
   float accelRoll  = calculateAccelerometerRoll(imuData.accelerometer);
   float accelPitch = calculateAccelerometerPitch(imuData.accelerometer);
   float gyroRoll   = calculateGyroscopeRoll(imuData.gyroscope, previousGyroData.roll, deltaTime);
   float gyroPitch  = calculateGyroscopePitch(imuData.gyroscope, previousGyroData.pitch, deltaTime);

   // calculate filtered roll and pitch values
   float roll  = calculateFilteredRoll(accelRoll, gyroRoll);
   float pitch = calculateFilteredPitch(accelPitch, gyroPitch);

   // use the filtered values for the tilt compensation for the magnetometer
   float magYaw  = calculateMagnetometerYaw(imuData.magnetometer, roll, pitch);
   float gyroYaw = calculateGyroscopeYaw(imuData.gyroscope, previousGyroData.yaw, deltaTime);
   float yaw     = calculateFilteredYaw(magYaw, gyroYaw);

   // get altitude data
   float altitude = altitudeSensor->getDistanceInches();

   // if 0 not giving accurate data so ignore and keep previous value
   // ping has limit of about 3m so if result in inches is > 115 assume it isn't accurate and use previous value
   previousGyroData.altitude = (altitude < 0.1 || altitude > 115.0) ? previousGyroData.altitude : altitude;

   // update the struct to pass back data
   orientation.roll     = roll;
   orientation.pitch    = pitch;
   orientation.yaw      = yaw;
   orientation.altitude = previousGyroData.altitude;

   return SUCCESS;
}

RETURN_CODE OrientationManager::getCurrentOrientation(float deltaTime, struct RPYData &filteredOrientation, struct RPYData &accel, struct RPYData &gyro, float &magYaw)
{
   // read imu to get the lastest data
   if (readIMU() != SUCCESS)
   {
      // some error occured
      Serial.println("Error reading IMU in OrientationManager::getCurrentOrientation()");
      return ERROR;
   }

   // calculate the roll and pitch angles
   float accelRoll  = calculateAccelerometerRoll(imuData.accelerometer);
   float accelPitch = calculateAccelerometerPitch(imuData.accelerometer);
   float gyroRoll   = calculateGyroscopeRoll(imuData.gyroscope, previousGyroData.roll, deltaTime);
   float gyroPitch  = calculateGyroscopePitch(imuData.gyroscope, previousGyroData.pitch, deltaTime);

   // calculate filtered roll and pitch values
   float roll  = calculateFilteredRoll(accelRoll, gyroRoll);
   float pitch = calculateFilteredPitch(accelPitch, gyroPitch);

   // use the filtered values for the tilt compensation for the magnetometer
   magYaw = calculateMagnetometerYaw(imuData.magnetometer, roll, pitch);
   float gyroYaw = calculateGyroscopeYaw(imuData.gyroscope, previousGyroData.yaw, deltaTime);
   float yaw     = calculateFilteredYaw(magYaw, gyroYaw);

   // get altitude data
   float altitude = altitudeSensor->getDistanceInches();

   // if 0 not giving accurate data so ignore and keep previous value
   previousGyroData.altitude = (altitude < 0.1) ? previousGyroData.altitude : altitude;

   // update the structs to pass back data
   filteredOrientation.roll  = roll;
   filteredOrientation.pitch = pitch;
   filteredOrientation.yaw   = yaw;
   filteredOrientation.altitude = previousGyroData.altitude;

   accel.pitch = accelPitch;
   accel.roll  = accelRoll;

   gyro.pitch = gyroPitch;
   gyro.roll  = gyroRoll;
   gyro.yaw   = gyroYaw;

   return SUCCESS;
}

