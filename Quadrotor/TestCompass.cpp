// 
// 
// 

#include "TestCompass.h"
#include "QuadrotorAPI.h"

const char *MATLAB_STOP_CONDITION = "StopTransmission";

compassTest::compassTest(struct Motors *motors, OrientationManager *orientationManager, ControllerManager *controllerManager, Bluetooth *bluetooth, HMC5883L *mag)
   : motors(motors), orientationManager(orientationManager), controllerManager(controllerManager), bluetooth(bluetooth), mag(mag)
{
	XYZData compassBearing;
	int xComp, yComp, zComp;
	mag->initialize();
	mag->selfTest();
	waitForStartCommand();
	for(int i =0; i < 11; i++)
	{
		setSpeed(10*i);
		for(int j=0; j<30; j++)
		{
			mag->readData(compassBearing);
			delay(10);
			xComp = compassBearing.X;
			yComp = compassBearing.Y;
			zComp = compassBearing.Z;
			Serial.write(xComp);
			Serial.write(yComp);
			Serial.write(zComp);
		}
	}
	shutdownSequence();
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

void compassTest::shutdownSequence()
{
   unsigned long currentTime;
	unsigned long previousTime = millis();
	float deltaTime;
	struct RPYData orientation;
	String message;
	int count = 0;
	// slowly decrease the height
	int height = controllerManager->getAltitudeController()->getSetpoint();
	while (height > 1)
	{
      currentTime = millis();
      if (currentTime - previousTime >= 30)
      {		
		   ++count;
         // get the delta time in seconds for calculations
         deltaTime = (currentTime - previousTime) / 1000.0;

			previousTime = currentTime;

         // get current orientation
         RETURN_CODE retVal = orientationManager->getCurrentOrientation(deltaTime, orientation);
			if (retVal == SUCCESS)
			{
			   // use the current orientation to adjust the motors
				controllerManager->flightControl(orientation, deltaTime);
				// check for commands via bluetooth
				message = bluetooth->readLine();

				// remove any whitespace
				message.trim();

				// handle the message if present
				if (!message.equals("")) handleMessage(message);

            // every half a second lower the quadrotor an inch
				if (count >= 16)
				{
					count = 0;
					controllerManager->getAltitudeController()->setSetpoint(--height);
				}
			}
      }
      // wait for 30ms to complete
	}
	// shut off motors
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

