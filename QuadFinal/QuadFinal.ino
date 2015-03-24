#include <Wire.h>
#include "Quad.h"

Quad test;

void setup()
{
	test.setup();
}

void loop()
{
	test.executeCycle();
}
