// I2C.cpp
#include <Wire.h>
#include "I2C.h"

void I2C::initialize()
{
   // set up i2c bus
   Wire.begin();
   delay(5);
}

RETURN_CODE I2C::writeDataToRegister(byte slaveAddress, byte regAddress, byte data)
{
   Wire.beginTransmission(slaveAddress);
   Wire.write(regAddress);
   Wire.write(data);
   return (RETURN_CODE)Wire.endTransmission();
}

RETURN_CODE I2C::writeDataToRegister(byte slaveAddress, byte regAddress)
{
   Wire.beginTransmission(slaveAddress);
   Wire.write(regAddress);
   return (RETURN_CODE)Wire.endTransmission();
}

RETURN_CODE I2C::readDataFromRegister(byte slaveAddress, byte regAddress, int numBytesRequested, byte buffer[], int &bytesRead)
{
   RETURN_CODE retVal;

   // send the register address to request data from
   retVal = writeDataToRegister(slaveAddress, regAddress);
   if (retVal != SUCCESS) return retVal;

   // request the data from the register
   Wire.beginTransmission(slaveAddress);
   Wire.requestFrom((int)slaveAddress, numBytesRequested);
   
   bytesRead = 0;
   while ( Wire.available() )
   {
      // make sure we don't overflow the buffer
      if (bytesRead >= numBytesRequested ) break;

      // read the byte and store in the buffer
      buffer[bytesRead++] = Wire.read();
   }

   retVal = (RETURN_CODE)Wire.endTransmission();

   return retVal;
}
