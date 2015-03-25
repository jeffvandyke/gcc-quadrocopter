// 
// 
// 

#include "TestCompass.h"
#include "QuadrotorAPI.h"

compassTest::compassTest(struct Motors *motors, Bluetooth *bluetooth, HMC5883L *mag)
   : motors(motors), bluetooth(bluetooth), mag(mag)
{
	XYZData compassBearing;
	int xComp, yComp, zComp, constant=1;
	mag->initialize();
	mag->selfTest();
//	motors->m1->initialize(CLOCKWISE, FOUR);

	// waits for a command from the GUI
	waitForStartCommand();
	
	//each iteration of this loop sets the duty cycle to a new value and then takes 50 samples over time.
	for(int i =0; i < 10; i++)
	{
		bluetooth->writeInt(i);
		//increases the duty cycle by 10% of the total
		setSpeed(10*i);
		//samples the data and sends it through the bluetooth to the computer
		for(int j=0; j<50; j++)	
		{
			//reads all of the data from the compass into a struct
			mag->readData(compassBearing);
			//ensures that the sampling data is measured at times that are somewhat far apart.  200 milliseconds.
			delay(200);
			//typcasts the compass readings to ints so that they can be sent.  Since the compass 
			//readings are in degrees the data loss is not large.
			xComp = compassBearing.X;
			yComp = compassBearing.Y;
			zComp = compassBearing.Z;

			//rights through the bluetooth
			bluetooth->writeInt(xComp);
			bluetooth->writeInt(yComp);
			bluetooth->writeInt(zComp);
		}
	}
	endTest();
}

void compassTest::handleMessage(String message)
{
   if (message.equals("Escape"))
   {
      Serial.println("Handled an escape");
      motors->m1->setSpeed(0.0);
      motors->m2->setSpeed(0.0);
      motors->m3->setSpeed(0.0);
      motors->m4->setSpeed(0.0);

      for (;;); // spin forever
   }
}

void compassTest::waitForStartCommand()
{
   String message = "";

   while (!message.equals("Start"))
   {
      delay(100);
      message = bluetooth->readLine();
      message.trim();
   }

   Serial.println("Received Start command");
}

void compassTest::endTest()
{
	int count = 0;
	// slowly decrease the height
	motors->m1->setSpeed(0.0);
	motors->m2->setSpeed(0.0);
	motors->m3->setSpeed(0.0);
	motors->m4->setSpeed(0.0);
}

void compassTest::setSpeed(int speed)
{
	motors->m1->setSpeed(speed);
    motors->m2->setSpeed(speed);
    motors->m3->setSpeed(speed);
    motors->m4->setSpeed(speed);
}

