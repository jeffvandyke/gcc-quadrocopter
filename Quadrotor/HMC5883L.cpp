// HMC588L.cpp 

#include "HMC5883L.h"
#include "I2C.h"

// device address
#define ADDRESS 0x1E

// register addresses
#define CONFIG_REG_A 0x00
#define CONFIG_REG_B 0x01
#define MODE_REG     0x02
#define DATAX0       0x03
#define STATUS       0x09

// data values
#define AVG_SAMPLES_8         0x60
#define DATA_OUTPUT_RATE_75HZ 0x18
#define CONT_MEASURE_MODE     0x00
#define SINGLE_MEASURE_MODE   0x01
#define DATA_READY            0x01

// read 6 bytes of data - x,y,z LSB and MSB
#define BYTES_TO_READ 6

HMC5883L::HMC5883L()
{
   // for HMC5883L default of 1.3 Ga range
   scaleFactor = 0.92;
}

void HMC5883L::initialize()
{
   // initialize sample rate
   i2c.writeDataToRegister(ADDRESS, CONFIG_REG_A, AVG_SAMPLES_8 | DATA_OUTPUT_RATE_75HZ);

   // adjust gain
   setRange(GAUSS_1_3);

   // continuous measurement mode
   i2c.writeDataToRegister(ADDRESS, MODE_REG, CONT_MEASURE_MODE);
}

RETURN_CODE HMC5883L::readData(struct XYZData &data)
{
   byte buffer[BYTES_TO_READ];
   RETURN_CODE retVal;
   int bytesRead;

   retVal = i2c.readDataFromRegister(ADDRESS, DATAX0, BYTES_TO_READ, buffer, bytesRead);
   if (retVal != SUCCESS)
   {
      // error occurred
      sendErrorMessage(retVal, "HMC5883L::readData()");
      return retVal;
   }

   // make sure it read all the data
   if (bytesRead != BYTES_TO_READ)
   {
      // expected data was not transmitted
      sendErrorMessage(INCOMPLETE_DATA, "HMC5883L::readData()");
      return INCOMPLETE_DATA;
   }

   // store data - combine MSB and LSB - adjust to correct scale
   // MSB is first followed by LSB
   data.X = ((int)buffer[1] | ((int)buffer[0] << 8)) * scaleFactor;
   data.Y = ((int)buffer[5] | ((int)buffer[4] << 8)) * scaleFactor;
   data.Z = ((int)buffer[3] | ((int)buffer[2] << 8)) * scaleFactor;

   return SUCCESS;
}

bool HMC5883L::dataAvailable()
{
   RETURN_CODE retVal;
   byte buffer;
   int bytesRead;

   retVal = i2c.readDataFromRegister(ADDRESS, STATUS, 1, &buffer, bytesRead);
   if (retVal != SUCCESS)
   {
      // error occurred
      sendErrorMessage(retVal, "HMC5883L::dataAvailable()");
      return false;
   }

   // return true if data is ready
   return buffer & DATA_READY;
}

RETURN_CODE HMC5883L::setRange(HMC5883L_SENSOR_FIELD_RANGE gauss)
{
   byte data = 0x00;

   switch (gauss)
   {
   case GAUSS_88:
      data = 0x00;
      scaleFactor = 0.73;
      break;
   case GAUSS_1_3:
      data = 0x01;
      scaleFactor = 0.92;
      break;
   case GAUSS_1_9:
      data = 0x02;
      scaleFactor = 1.22;
      break;
   case GAUSS_2_5:
      data = 0x03;
      scaleFactor = 1.52;
      break;
   case GAUSS_4_0:
      data = 0x04;
      scaleFactor = 2.27;
      break;
   case GAUSS_4_7:
      data = 0x05;
      scaleFactor = 2.56;
      break;
   case GAUSS_5_6:
      data = 0x06;
      scaleFactor = 3.03;
      break;
   case GAUSS_8_1:
      data = 0x07;
      scaleFactor = 4.35;
      break;
   default:
      // not a valid option
      sendErrorMessage(INVALID, "HMC5883L::setRange()");
      return INVALID;
   }
    
   // Setting is in the top 3 bits of the register.
   return i2c.writeDataToRegister(ADDRESS, CONFIG_REG_B, data << 5);
}

void HMC5883L::selfTest()
{
   struct XYZData data;   
   i2c.writeDataToRegister(ADDRESS, CONFIG_REG_A, 0x71);
   i2c.writeDataToRegister(ADDRESS, CONFIG_REG_B, 0x05 << 5);
   i2c.writeDataToRegister(ADDRESS, MODE_REG, 0x00);
   delay(10);
   for (int i=0; i<2; ++i)
   {
   readData(data);

   Serial.print("compass x: ");
   Serial.println(data.X);
   Serial.print("compass y: ");
   Serial.println(data.Y);
   Serial.print("compass z: ");
   Serial.println(data.Z);

   delay(70);
   }

   i2c.writeDataToRegister(ADDRESS, CONFIG_REG_A, 0x70);
}


