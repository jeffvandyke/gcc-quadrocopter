/************************************************/
/* Written by:  Zachary Wontrop                 */
/* Date:        1/26/14                         */
/* Name:        Ping.h                          */
/* Description: This class provides an          */
/*    interface for using the PING))) sensor    */
/************************************************/

#ifndef _PING_h
#define _PING_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "QuadrotorAPI.h"

class Ping
{
public:
   // the pin it is connected to on the Arduino
   Ping(int pin);
   ~Ping();

   float getDistanceInches();

private:
   int pin;
};


#endif