// SparkFun9DOF.cpp

#include "SparkFun9DOF.h"

SparkFun9DOF::SparkFun9DOF()
{
   accelerometer = new ADXL345;
   gyroscope     = new ITG3200;
   magnetometer  = new HMC5883L;
}

SparkFun9DOF::~SparkFun9DOF()
{
   if (accelerometer) delete accelerometer;
   if (gyroscope)     delete gyroscope;
   if (magnetometer)  delete magnetometer;
}

void SparkFun9DOF::initialize()
{
   accelerometer->initialize();
   gyroscope->initialize();
   magnetometer->initialize();
}

RETURN_CODE SparkFun9DOF::readData(struct IMUData &data)
{
   RETURN_CODE retVal;

   // check if the sensors are updated with new data since the previous read
   // TODO: checking for available data can probably be eliminated since in the control loop enough time
   // will be taken up in the calcuations that it will have updated itself in the meantime
   // this might speed up this read process which currently takes about 6 ms to read all 3 sensors
   //while ( !accelerometer->dataAvailable() ) delayMicroseconds(100);
   retVal = accelerometer->readData(data.accelerometer);
   if (retVal != SUCCESS)
   {
      // error occurred
      sendErrorMessage(retVal, "SparkFun9DOF::readData()");
      return retVal;
   }

   //while ( !magnetometer->dataAvailable() ) delayMicroseconds(100);
   retVal = magnetometer->readData(data.magnetometer);
   if (retVal != SUCCESS)
   {
      // error occurred
      sendErrorMessage(retVal, "SparkFun9DOF::readData()");
      return retVal;
   }

   //while ( !gyroscope->dataAvailable() ) delayMicroseconds(100);
   retVal = gyroscope->readData(data.gyroscope);
   if (retVal != SUCCESS)
   {
      // error occurred
      sendErrorMessage(retVal, "SparkFun9DOF::readData()");
      return retVal;
   }

   return SUCCESS;
}


