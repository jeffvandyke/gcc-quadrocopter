/************************************************/
/* Written by:  Aaron Derstine                  */
/* Date:        1/11/14                         */
/* Name:        QuadrotorAPI.h                  */
/* Description: This file defines different     */
/*    functions and structures used in the      */
/*    quadrotor.                                */
/************************************************/

#ifndef _QUADROTORAPI_h
#define _QUADROTORAPI_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

// defines the different codes that functions return to indicate operation results
enum RETURN_CODE
{
   // The first 5 codes are from Arduino and the Wire library used for I2C
   // we used these as the basis for error codes in debugging and indicating success
   SUCCESS,         // 0
   OVERFLOW,        // 1
   NACK_ADDRESS,    // 2
   NACK_DATA,       // 3
   ERROR,           // 4
   INCOMPLETE_DATA, // 5
   INVALID,         // 6
   UNINITIALIZED    // 7
};

// sends an error message using the Arduino serial library
// Notes:
//    Requires that Serial.begin() is called before using which is done in quadrotor::initialize()
// Inputs:
//    code         - the RETURN_CODE indicating the error
//    functionName - the function in which the error occured
// Returns:
//    none
void sendErrorMessage(RETURN_CODE code, const char *functionName);

// stores XYZ value data
struct XYZData
{
   float X;
   float Y;
   float Z;
};

// stores roll, pitch, yaw data in degrees - also altitude in inches
struct RPYData
{
   float roll;
   float pitch;
   float yaw;
   float altitude;
};

#endif