/*
~cornelius Kloosterman
This file defines an object that calculates and stores trigonometric values
so that they can be retrieved with minimal clock cycles. Ouputs ratios of 
arc functions as being multiplied by 100 to minimize clock cycles used 
outside of the the setup loop.
*/

#ifndef TRIG_h
#define TRIG_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class trig
{
	void initialize();
	int getArctan2();
	int  getCos();
	int getSin();
};