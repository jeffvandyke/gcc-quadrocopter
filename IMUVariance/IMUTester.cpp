#include "IMUTester.h"
#include <Wire.h>
#include "I2C.h"
#include "Bluetooth.h"

#define MOTOR_OUT 6
#define MOTOR_TRIG 53
#define OUTPIN 6
#define TRIGPIN 53

IMUTester::IMUTester(void)
{
	i2c = I2C();
	return;
}

IMUTester::~IMUTester(void)
{
}

int motorInitialize()
{
	pinMode(OUTPIN,OUTPUT);
	pinMode(TRIGPIN,OUTPUT);
	digitalWrite(TRIGPIN,LOW);
	analogWrite(OUTPIN,0);
	delay(3000);
	analogWrite(OUTPIN,100);
	delay(1000);
	analogWrite(OUTPIN,0);
	delay(200);

	return 0;
}

int motorSet(int dutyCycle)
{
	digitalWrite(TRIGPIN,HIGH);
	analogWrite(OUTPIN,dutyCycle);
	digitalWrite(TRIGPIN,LOW);

	Serial.print("Duty cycle set to ");
	Serial.print(dutyCycle);
	Serial.print("\n");

	return 0;
}

int IMUTester::setupTester(void)
{
	sensorMode = ALL;
	i2c.initialize();
	Serial.begin(115200);
	Serial.print("IMUTester::setupTester called");
	//acc.setup();
	accel.begin();
	dataRate_t maxDataRate = ADXL345_DATARATE_3200_HZ;
	accel.setDataRate(maxDataRate);

	gyro.setup();
	bar.setup();
	comp.setup();
	//gpssetup {
	Serial2.begin(57600);
	delay(1000);
	finish = millis()+10;
	//}
	   Serial1.begin(115200, SERIAL_8N1); // baud rate = 115200, 8 data bits, no parity, 1 stop bit
   Serial1.println("R,1");

   
	Serial.print("a.x,a.y,a.z,g.x,g.y,g.z,c.x,c.y,G.x,G.y,G.z,G.a");
	return 0;
}

int IMUTester::executeCycle(void)

{
	bool newData = false;

	this->readSerialCommand();

	int sensorData;
	finish = millis();

	while (Serial2.available() )//&& millis() < finish)
     {
		char c = Serial2.read();
	    //Serial.print(c);
		if(gps.encode(c))
		{
			newData = true;
		}
	}
	Serial.print(accel.getDataRate() + "\n");
	

	/*dataToSend = "***"+((int)acc.readRawX());
	dataToSend += ","+acc.readRawY();
	dataToSend += ","+acc.readRawZ();
	dataToSend += ","+gyro.readRawX();
	dataToSend += ","+gyro.readRawY();
	dataToSend += ","+gyro.readRawZ();
	dataToSend += ","+comp.getRawX();
	dataToSend += ","+comp.getRawY();*/
	

			//Serial.print("Accelerometer: \n");	
	gyro.readRawX();
		//Serial.print(accel.getX());  Serial.print("\n");
		//Serial1.write(acc.readRawX());           //Serial.print(",");
		//Serial.print(acc.readRawY());           //Serial.print(",");
		//Serial1.write(acc.readRawZ());           //Serial.print(",");
		//Serial1.print("\n");
		//Serial.print("Gyroscope: \n");
		//Serial.print(gyro.readRawX());          Serial.print(",");
		//Serial.print(gyro.readRawY());          Serial.print(",");
		//Serial.print(gyro.readRawZ());          Serial.print(",");
		//Serial.print("Compass: \n");
		//Serial.print(comp.getRawX());           Serial.print(",");
		//Serial.print(comp.getRawY());           Serial.print(",");
		//Serial.print(comp.getRawZ());			Serial.print("\n");
		//Serial.print("GPS: \n");
		//Serial.print(gps.readRawLat());			Serial.print(", ");
		//Serial.print(gps.readRawLong());		
		//Serial.print("\n");

		gps.get_datetime(&date, &time);
	if (newData && time != oldTime)
	{
			gps.get_position(&newLong, &newLat); //, &fixAge);

			/*dataToSend += ","+newLat; 
			dataToSend += ","+newLong;
			dataToSend += ","+gps.altitude();
			dataToSend += ",1\n";*/
	
			Serial.print(newLat);  Serial.print(",");
			Serial.print(newLong); Serial.print(",");
			Serial.print(gps.altitude()); Serial.print(",1");
			oldTime = time;

	}
		else
		{
			//Serial.print("0,0,0,0\n");
		}
		Serial.print (millis()-finish); Serial.print("\n");
	finish = millis();

	//Serial.print(dataToSend);
	while(finish>millis())
	{
		delay(1);
	}
	finish = finish + 10;
	return 0;
}

void IMUTester::readSerialCommand() {
	int serialData = Serial.read();

	if (serialData != -1)
		// clear some space
			Serial.print("\n\n\n");

	switch (serialData)
	{
	case (int)'1':
		motorSet(100);
		break;
	case (int)'2':
		motorSet(120);
		break;
	case (int)'3':
		motorSet(130);
		break;
	case (int)'4':
		motorSet(140);
		break;
	case (int)'5':
		motorSet(150);
		break;
	case (int)'6':
		motorSet(160);
		break;
	case (int)'7':
		motorSet(170);
		break;
	case (int)'8':
		motorSet(180);
		break;
	case (int)'9':
		motorSet(190);
		break;
	case (int)'10':
		motorSet(200);
		break;
	case (int)'11':
		motorSet(210);
		break;
	case (int)'12':
		motorSet(220);
		break;
	case -1:
		break;
	default:
		Serial.print("unknown command: ");
		Serial.println((char)serialData);
	}
}