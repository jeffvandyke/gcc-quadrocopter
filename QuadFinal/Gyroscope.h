#pragma once
#include "ITG3200.h"

class Gyroscope
{
public:
	Gyroscope(void);
	~Gyroscope(void);

	ITG3200 gyro;

	int readRawX(void);
	int readRawY(void);
	int readRawZ(void);
	int setup(void);
};




