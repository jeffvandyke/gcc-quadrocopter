// ITG3200.cpp

#include "ITG3200.h"
#include "I2C.h"

// device address
#define ADDRESS 0x68

// register addresses
#define SAMPLE_RATE_DIVIDER   0x15
#define SAMPLE_CONFIG         0x16
#define INT_CONFIG            0x17
#define STATUS                0x1A
#define TEMP_OUT_H            0x1B
#define GYRO_XOUT_H           0x1D
#define POWER_SETTINGS        0x3E

// data values
#define RAW_DATA_READY        0x01  // data is available - used for enable and read
#define PLL_XGYRO_REF         0x01  // use the X Gyro for internal clock reference

// read 6 bytes of data - x,y,z LSB and MSB
#define BYTES_TO_READ 6

// read 2 bytes for temperature data
#define TEMP_DATA_SIZE 2

#define GYRO_SENSITIVITY 14.375

void ITG3200::initialize()
{
   // set up full scale range and internal sample rate
   setDLPFConfiguration(BW_256HZ);

   // set up sample rate divisor
   setSampleRateDivider(0x32);

   // set up interrupt on data available
   i2c.writeDataToRegister(ADDRESS, INT_CONFIG, RAW_DATA_READY);

   // select internal clock reference
   i2c.writeDataToRegister(ADDRESS, POWER_SETTINGS, PLL_XGYRO_REF);

   // wait 50ms for settings to take effect
   delay(50);
}

RETURN_CODE ITG3200::readData(struct XYZData &data)
{
   byte buffer[BYTES_TO_READ];
   RETURN_CODE retVal;
   int bytesRead;

   retVal = i2c.readDataFromRegister(ADDRESS, GYRO_XOUT_H, BYTES_TO_READ, buffer, bytesRead);
   if (retVal != SUCCESS)
   {
      // error occurred
      sendErrorMessage(retVal, "ITG3200::readData()");
      return retVal;
   }

   // make sure it read all the data
   if (bytesRead != BYTES_TO_READ)
   {
      // expected data was not transmitted
      sendErrorMessage(INCOMPLETE_DATA, "ITG3200::readData()");
      return INCOMPLETE_DATA;
   }

   // store data - combine MSB and LSB
   // MSB is first followed by LSB
   data.X = ((int)buffer[1] | ((int)buffer[0] << 8)) / GYRO_SENSITIVITY;
   data.Y = ((int)buffer[3] | ((int)buffer[2] << 8)) / GYRO_SENSITIVITY;
   data.Z = ((int)buffer[5] | ((int)buffer[4] << 8)) / GYRO_SENSITIVITY;

   return SUCCESS;
}

RETURN_CODE ITG3200::readTemperature(float &temp)
{
   byte buffer[TEMP_DATA_SIZE];
   RETURN_CODE retVal;
   int bytesRead;

   retVal = i2c.readDataFromRegister(ADDRESS, TEMP_OUT_H, TEMP_DATA_SIZE, buffer, bytesRead);
   if (retVal != SUCCESS)
   {
      // error occurred
      sendErrorMessage(retVal, "ITG3200::readTemperature()");
      return retVal;
   }

   // make sure it read all the data
   if (bytesRead != TEMP_DATA_SIZE)
   {
      // expected data was not transmitted
      sendErrorMessage(INCOMPLETE_DATA, "ITG3200::readTemperature()");
      return INCOMPLETE_DATA;
   }

   // store data - combine MSB and LSB
   // MSB is first followed by LSB
   int rawTemp = (int)buffer[1] | ((int)buffer[0] << 8);

   // adjust to find temp in celsius
   temp = 35.0 + (rawTemp + 13200)/280.0;

   return SUCCESS;
}

bool ITG3200::dataAvailable()
{
   RETURN_CODE retVal;
   byte buffer;
   int bytesRead;

   retVal = i2c.readDataFromRegister(ADDRESS, STATUS, 1, &buffer, bytesRead);
   if (retVal != SUCCESS)
   {
      // error occurred
      sendErrorMessage(retVal, "ITG3200::dataAvailable()");
      return false;
   }

   // return true if data is ready
   return buffer & RAW_DATA_READY;
}

RETURN_CODE ITG3200::setSampleRateDivider(byte divider)
{
   // set up sample rate divisor
   return i2c.writeDataToRegister(ADDRESS, SAMPLE_RATE_DIVIDER, divider); // Fsample = Finternal / (divider + 1)
}

RETURN_CODE ITG3200::setDLPFConfiguration(ITG3200_DLPF_CONFIG bw)
{
   byte data = 0x00;

   switch (bw)
   {
   case BW_256HZ:
      data = 0x00;
      break;
   case BW_188HZ:
      data = 0x01;
      break;
   case BW_98HZ:
      data = 0x02;
      break;
   case BW_42HZ:
      data = 0x03;
      break;
   case BW_20HZ:
      data = 0x04;
      break;
   case BW_10HZ:
      data = 0x05;
      break;
   case BW_5HZ:
      data = 0x06;
      break;
   default:
      // not a valid option
      sendErrorMessage(INVALID, "ITG3200::setDLPFConfiguration()");
      return INVALID;
   }

   // set full-scale range of the gyro sensors - needed for proper operation
   data |= 0x18;

   // set up full scale range and internal sample rate
   return i2c.writeDataToRegister(ADDRESS, SAMPLE_CONFIG, data);
}