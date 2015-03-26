/*
==============================================================
 Trig Library
==============================================================
 Original Code (C) 2012-2013 Oscar Liang
 Ported September 2014 with permission from original author
 Licensed under the MIT licence.
 
 This is a library to quickly do common trigonometry functions
 on Arduinos/other microprocessors.
 
 This is the main header file.
==============================================================
*/
Trig trig;
//Include Guard:
#ifndef Trig_h
#define Trig_h

//Arduino libraries:
// #include "Arduino.h"
#include <bitset>

class Trig {
  public:
		Trig();
		int radToMicro(float rad);
		int radToDeg(float rad);
		int floatToInt(float input);
		int sin(int deg);
		int cos(int deg);
		float acos(float num);
		float atan2(float opp, float adj);
		int atan2(int opp, int adj);
	private:
		int floatToBitShiftInt(float num);
		float bitShiftIntToFloat(int num);
}

//Create class to use:


#endif //#ifndef Trig_h
