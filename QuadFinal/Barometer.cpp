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
	Serial1.begin(4800);
	bool result = barom.begin();
	if (result){
		Serial.println("Barometer 'begin'ed fine");
	} else {
		Serial.println("Barometer 'begin'ed FAILED");
	}
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
