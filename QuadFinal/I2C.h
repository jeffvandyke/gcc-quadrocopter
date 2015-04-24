/**********************************************/
/* Written by:  Aaron Derstine                */
/* Date:        1/11/14                       */
/* Name:        I2C.h                         */
/* Description: This class encapsulates       */
/*    the Arduino Wire library and simplifies */
/*    reading and writing data via I2C        */
/*    Creates a global object to be used      */
/*    anywhere in the project                 */
/**********************************************/

#ifndef _I2C_h
#define _I2C_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif



enum RETURN_CODE
{
   // The first 5 codes are from Arduino and the Wire library used for I2C
   // we used these as the basis for error codes in debugging and indicating success
   SUCCESS,         // 0
   OVERFLOW_,        // 1
   NACK_ADDRESS,    // 2
   NACK_DATA,       // 3
   ERROR,           // 4
   INCOMPLETE_DATA, // 5
   INVALID,         // 6
   UNINITIALIZED    // 7
};

class I2C
{
public:
   // Initializes the I2C interface via the Wire library
   // Notes:
   //    Should be called during the one-time initialization sequence of the quadrotor
   //    before any communication takes place
   // Inputs:
   //    none
   // Returns:
   //    none
   void initialize();

   // Writes a byte of data to a register
   // Inputs:
   //    slaveAddress - address of the slave device
   //    regAddress   - address of the register to be written to (in the slave device)
   //    data         - single byte of data to be written
   // Returns:
   //    RETURN_CODE indicating result of transmission
   RETURN_CODE writeDataToRegister(byte slaveAddress, byte regAddress, byte data);

   // Reads data from a register
   // Notes:
   //    Before calling this function a buffer needs to be allocated to store the data
   //    The base address of this array is passed in as a parameter
   // Inputs:
   //    slaveAddress      - address of the slave device
   //    regAddress        - address of the register to be read from (in the slave device)
   //    numBytesRequested - number of bytes expected to read (size of the buffer)
   //    buffer            - base address of an array used to store the data received
   //    bytesRead         - actual number of bytes received (passed by reference so modified by the function)
   // Returns:
   //    RETURN_CODE indicating result of transmission
   RETURN_CODE readDataFromRegister(byte slaveAddress, byte regAddress, int numBytesRequested, byte buffer[], int &bytesRead);

private:
   // used for readDataFromRegister since we only need to send the register address to read from and there is no
   // additional data to write
   RETURN_CODE writeDataToRegister(byte slaveAddress, byte regAddress);
};


#endif




