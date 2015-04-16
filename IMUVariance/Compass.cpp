#include "Compass.h"


Compass::Compass(void)
{
}


Compass::~Compass(void)
{
}


int Compass::setup(void)
{
	comp.initialize();
	return 0;
}


int Compass::getRawX(void)
{
	return comp.getHeadingX();
}


int Compass::getRawY(void)
{
	return comp.getHeadingY();
}


int Compass::getRawZ(void)
{
	return comp.getHeadingZ();
}
