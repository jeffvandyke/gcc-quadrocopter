/************************************************/
/* Written by:  Aaron Derstine                  */
/* Date:        1/23/14                         */
/* Name:        Bluetooth.h                     */
/* Description: This class provides an          */
/*    interface to communicate through the      */
/*    Sparkfun BlueSMiRF bluetooth modem        */
/************************************************/

#ifndef _BLUETOOTH_h
#define _BLUETOOTH_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "QuadrotorAPI.h"

class Bluetooth
{
public:
   ~Bluetooth();

   void initialize();

   String readLine();

   void writeLine(String dataOut);

   void writeInt(int dataOut);

   void calibrateLoop();

private:
   int dataIn;
};

#endif

