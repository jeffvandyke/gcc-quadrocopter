#include "Adafruit_Sensor.h"
#include <Wire.h>
#include "IMUTester.h"

IMUTester tester;

void setup()
{
  tester.setupTester();
}

void loop()
{
  //Serial.println("vindaloop");
  tester.executeCycle();
}
