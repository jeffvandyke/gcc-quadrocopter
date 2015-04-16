#include "Barometer.h"


Barometer::Barometer(void)
{
	
}


Barometer::~Barometer(void)
{
}


int Barometer::setup(void)
{
	barom = Adafruit_BMP085_Unified(10085);

	return 0;
}


int Barometer::readRawAltitude(void)
{
	float pressure;
	float temperature;
	barom.getPressure(&pressure);
	barom.getTemperature(&temperature);
	float altitude = barom.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA,pressure);
	//Serial.println(altitude);
	return static_cast<int>(altitude);
}
