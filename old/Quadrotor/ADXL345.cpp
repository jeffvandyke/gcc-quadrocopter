// ADXL345.cpp

#include "ADXL345.h"
#include "I2C.h"

// device address
#define ADDRESS 0x53

// register addresses
#define BW_RATE     0x2C
#define POWER_CTL   0x2D
#define INT_SOURCE  0x30
#define DATA_FORMAT 0x31
#define DATAX0      0x32

// data values
#define MEASURE_MODE_ON 0x08
#define DATA_READY      0x80

// read 6 bytes of data - x,y,z LSB and MSB
# define BYTES_TO_READ 6

ADXL345::ADXL345()
{
   // default 2G range
   scaleFactor = 2.0 / 512.0;
}

void ADXL345::initialize()
{
   // set sensitivity range
   setRange(RANGE_2G);

   // set output rate
   setSampleRate(NORMAL_POWER, RATE_200HZ);

   // set measure mode on
   enableMeasurements();
}

RETURN_CODE ADXL345::readData(struct XYZData &data)
{
   byte buffer[BYTES_TO_READ];
   RETURN_CODE retVal;
   int bytesRead;

   retVal = i2c.readDataFromRegister(ADDRESS, DATAX0, BYTES_TO_READ, buffer, bytesRead);
   if (retVal != SUCCESS)
   {
      // error occurred
      sendErrorMessage(retVal, "ADXL345::readData()");
      return retVal;
   }

   // make sure it read all the data
   if (bytesRead != BYTES_TO_READ)
   {
      // expected data was not transmitted
      sendErrorMessage(INCOMPLETE_DATA, "ADXL345::readData()");
      return INCOMPLETE_DATA;
   }

   // store data - combine MSB and LSB
   // LSB is first followed by MSB
   data.X = ((int)buffer[0] | ((int)buffer[1] << 8)) * scaleFactor;
   data.Y = ((int)buffer[2] | ((int)buffer[3] << 8)) * scaleFactor;
   data.Z = ((int)buffer[4] | ((int)buffer[5] << 8)) * scaleFactor;

   return SUCCESS;
}

bool ADXL345::dataAvailable()
{
   RETURN_CODE retVal;
   byte buffer;
   int bytesRead;

   retVal = i2c.readDataFromRegister(ADDRESS, INT_SOURCE, 1, &buffer, bytesRead);
   if (retVal != SUCCESS)
   {
      // error occurred
      sendErrorMessage(retVal, "ADXL345::dataAvailable()");
      return false;
   }

   // return true if data is ready
   return buffer & DATA_READY;
}

RETURN_CODE ADXL345::setSampleRate(ADXL345_POWER_MODE powerMode, ADXL345_BW_RATE sampleRate)
{
   byte data = 0x00;

   switch (sampleRate)
   {
   case RATE_3200HZ:
      data = 0x0F;
      break;
   case RATE_1600HZ:
      data = 0x0E;
      break;
   case RATE_800HZ:
      data = 0x0D;
      break;
   case RATE_400HZ:
      data = 0x0C;
      break;
   case RATE_200HZ:
      data = 0x0B;
      break;
   case RATE_100HZ:
      data = 0x0A;
      break;
   case RATE_50HZ:
      data = 0x09;
      break;
   case RATE_25HZ:
      data = 0x08;
      break;
   default:
      // not a valid option
      sendErrorMessage(INVALID, "ADXL345::setSampleRate()");
      return INVALID;
   }

   // set bit indicating low power mode if needed
   if (powerMode == LOW_POWER) data |= 0x10;

   return i2c.writeDataToRegister(ADDRESS, POWER_CTL, data);
}

RETURN_CODE ADXL345::setRange(ADXL345_G_RANGE range)
{
   byte data = 0x00;

   // scale factor = range/512
   switch (range)
   {
   case RANGE_2G:
      data = 0x00;
      scaleFactor = 2.0 / 512.0;
      break;
   case RANGE_4G:
      data = 0x01;
      scaleFactor = 4.0 / 512.0;
      break;
   case RANGE_8G:
      data = 0x02;
      scaleFactor = 8.0 / 512.0;
      break;
   case RANGE_16G:
      data = 0x03;
      scaleFactor = 16.0 / 512.0;
      break;
   default:
      // not a valid option
      sendErrorMessage(INVALID, "ADXL345::setRange()");
      return INVALID;
   }

   return i2c.writeDataToRegister(ADDRESS, DATA_FORMAT, data);
}

RETURN_CODE ADXL345::enableMeasurements()
{
   return i2c.writeDataToRegister(ADDRESS, POWER_CTL, MEASURE_MODE_ON);
}


