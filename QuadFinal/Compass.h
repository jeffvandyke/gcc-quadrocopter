#pragma once
#include "HMC5883L.h"
class Compass
{
public:
	Compass(void);
	~Compass(void);

	HMC5883L comp;

	int setup(void);
	int getRawX(void);
	int getRawY(void);
	int getRawZ(void);
};




