#include "IMUTester.h"
#include <Wire.h>
#include "I2C.h"
#include "Bluetooth.h"

#define MOTOR_OUT 6
#define MOTOR_TRIG 53
#define OUTPIN 6
#define TRIGPIN 53


#define SERIAL_USB
//#define SERIAL_BLUETOOTH


IMUTester::IMUTester(void)
{
	i2c = I2C();
	return;
}

IMUTester::~IMUTester(void)
{
}


int IMUTester::setupTester(void)
{
#ifdef SERIAL_USB
	Serial.begin(115200);
	sensorMode = ALL;
#endif
#ifdef SERIAL_BLUETOOTH
	Serial1.begin(115200, SERIAL_8N1); // baud rate = 115200, 8 data bits, no parity, 1 stop bit
	Serial1.println("R,1"); // restart it
#endif


	i2c.initialize();
	serialPrint("IMUTester::setupTester called");
	//acc.setup();
	accel.begin();

	gyro.initialize();

	//dataRate_t maxDataRate = ADXL345_DATARATE_3200_HZ;
	//accel.setDataRate(maxDataRate);
	//gyro.setRate(2);

	delay(1000);

	//bar.setup();
	//comp.setup();
	//gpssetup {
	Serial2.begin(57600);
	delay(1000);

	//}
//	   Serial1.begin(115200, SERIAL_8N1); // baud rate = 115200, 8 data bits, no parity, 1 stop bit
//   Serial1.println("R,1");

	cycles = 1;
	sumCollectionTime = 5;
	serialPrint("a.x,a.y,a.z,g.x,g.y,g.z,c.x,c.y,G.x,G.y,G.z,G.a");
	finish = millis() + 100;
	return 0;
}

int IMUTester::executeCycle(void)
{
	bool newData = false;

	this->readSerialCommand();

	int sensorData;
	finish = finish + 20;

	stopTime = finish - (sumCollectionTime / cycles);
	while (millis() <= stopTime)
	{
		while (Serial2.available())//&& millis() < finish)
		{
			char c = Serial2.read();
			//serialPrint(c);
			if (gps.encode(c))
				newData = true;
		}
	}

	sensorCollectionStart = millis();
	

	/*
	if (reportGPS)
	{
		serialPrint("GPS: ");  printGPS(); serialPrintln(finish);

	}

	if (reportAccel)
	{
		serialPrint("Accelerometer: "); serialPrintln(timeAccel());
	}

	if (reportGyro)
	{
		serialPrint("Gyroscope: "); serialPrintln(timeGyro());
	}
	if (reportGyroAll)
	{
		serialPrint("Gyroscope2: "); serialPrintln(timeGyroAll());
	}

	if (reportComp)
	{
		serialPrint("Compass: "); serialPrintln(timeComp());
	}*/

			//serialPrint("Accelerometer: \n");	
	//gyro.readRawX();
		//serialPrint(accel.getX());  serialPrint("\n");
		serialPrint(accel.getX());           serialPrint(",");
		serialPrint(accel.getY());           serialPrint(",");
		serialPrint(accel.getZ());           serialPrint(",");
		//Serial1.print("\n");
		//serialPrint("Gyroscope: \n");
		serialPrint(gyro.getRotationX());          serialPrint(",");
		serialPrint(gyro.getRotationY());          serialPrint(",");
		serialPrint(gyro.getRotationZ());          serialPrint(",");
		//serialPrint("Compass: \n");
		//serialPrint(comp.getRawX());           serialPrint(",");
		//serialPrint(comp.getRawY());           serialPrint(",");
		//serialPrint(comp.getRawZ());			serialPrint("\n");
		serialPrint("0,0,");

		gps.get_datetime(&date, &time);
	if (newData && time != oldTime)
	{
			gps.get_position(&newLong, &newLat); //, &fixAge);

			serialPrint(newLat);  serialPrint(",");
			serialPrint(newLong); serialPrint(",");
			serialPrint(gps.altitude()); serialPrintln(",1");
			oldTime = time;

	}
		else
		{
			serialPrintln("0,0,0,0");
		}
	sumCollectionTime += (millis() - sensorCollectionStart);
	cycles++;

	return 0;
}

void IMUTester::printGPS()
{
	serialPrint(gps.readRawLat()); serialPrint(",");
	serialPrint(gps.readRawLong()); serialPrint(",");
	serialPrint(gps.altitude()); serialPrint(",");
	return;
}

int IMUTester::timeGyro()
{
	finish = millis();
	serialPrint(gyro.getRotationX()); serialPrint(",");
	serialPrint(gyro.getRotationY()); serialPrint(",");
	serialPrint(gyro.getRotationZ()); serialPrint(",");
	return millis()-finish;	
}

int IMUTester::timeGyroAll()
{
	int16_t a, b, c;
	finish = millis();
	gyro.getRotation(&a, &b, &c);
	serialPrint(a); serialPrint(",");
	serialPrint(b); serialPrint(",");
	serialPrint(c); serialPrint(",");
	return millis() - finish;
}

int IMUTester::timeAccel()
{
	finish = millis();
	serialPrint(accel.getX()); serialPrint(",");
	serialPrint(accel.getY()); serialPrint(",");
	serialPrint(accel.getZ()); serialPrint(",");
	return millis() - finish;

}

int IMUTester::timeComp()
{
	finish = millis();
	serialPrint(comp.getRawX()); serialPrint(",");
	serialPrint(comp.getRawY()); serialPrint(",");
	return millis() - finish;

}

inline bool toggle(bool var)
{
	return var ? false : true;
}

void IMUTester::readSerialCommand() {
	int serialData = Serial.read();

	if (serialData != -1)
		// clear some space
		serialPrint("\n\n\n");

	switch (serialData)
	{
	case -1:
		break;
	case 'i':
		//initialize(Serial.read());
		break;
	case 'a':
		reportAccel = toggle(reportAccel);
		break;
	case 'y':
		reportGyro = toggle(reportGyro);
		break;
	case 'Y':
		reportGyroAll = toggle(reportGyroAll);
		break;
	case 'g':
		reportGPS = toggle(reportGPS);
		break;
	case 'c':
		reportComp = toggle(reportComp);
		break;

	default:
		serialPrint("unknown command: ");
		serialPrintln(serialData);
	}
}
/*
void IMUTester::initialize(int sensor)
{
	switch (sensor)
	{
	case -1:
		break;
	case 'a':
		reportAccel = toggle(reportAccel);
		break;
	case 'g':
		reportGyro = toggle(reportGyro);
		break;
	case 'c':
		reportComp = toggle(reportComp);
		break;
	default:
		serialPrint("unknown sensor: "); serialPrintln(sensor);
	}
}*/


void IMUTester::serialPrint(String val)
{
	#ifdef SERIAL_USB
		Serial.print(val);
	#endif
	#ifdef SERIAL_BLUETOOTH
		Serial1.print(val);
	#endif
}

void IMUTester::serialPrint(int val)
{
	#ifdef SERIAL_USB
		Serial.print(val);
	#endif
	#ifdef SERIAL_BLUETOOTH
		Serial1.print(val);
	#endif
}

void IMUTester::serialPrintln(String val)
{
	#ifdef SERIAL_USB
		Serial.println(val);
	#endif
	#ifdef SERIAL_BLUETOOTH
		Serial1.println(val);
	#endif
}

void IMUTester::serialPrintln(int val)
{
	#ifdef SERIAL_USB
		Serial.println(val);
	#endif
	#ifdef SERIAL_BLUETOOTH
		Serial1.println(val);
	#endif
}