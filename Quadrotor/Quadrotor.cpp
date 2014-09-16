// Quadrotor.cpp

#include "I2C.h"
#include "Quadrotor.h"
#include "Motor.h"
#include "SparkFun9DOF.h"

#include "Test.h"

// comment out if not testing
//#define TESTING

#define SAMPLE_RATE 30


Quadrotor::Quadrotor()
{
   imu    = new SparkFun9DOF;
   motor1 = new Motor;
   motor2 = new Motor;
   motor3 = new Motor;
   motor4 = new Motor;

   orientationManager = new OrientationManager;
   controller         = new ControllerManager;
   bluetooth          = new Bluetooth;
   altitudeSensor     = new Ping(22);

   message = "";
   previousTime = 0;
   currentTime = 0;
   deltaTime = 0.0;
}

Quadrotor::~Quadrotor()
{
   if (imu)    delete imu;
   if (motor1) delete motor1;
   if (motor2) delete motor2;
   if (motor3) delete motor3;
   if (motor4) delete motor4;

   if (orientationManager) delete orientationManager;
   if (controller)         delete controller;
   if (bluetooth)          delete bluetooth;
   if (altitudeSensor)	   delete altitudeSensor;
}

// one-time initialization routine
void Quadrotor::initialize()
{ 
   // wait 1 sec for devices to power on
   delay(1000);

   // set up Serial output via USB
   Serial.begin(115200);
   delay(5);

   // initialize i2c bus
   i2c.initialize();

   // initialize bluetooth
   bluetooth->initialize();

   // initialize Motors
   motor1->initialize(CLOCKWISE, ONE);
   motor2->initialize(COUNTER_CLOCKWISE, TWO);
   motor3->initialize(CLOCKWISE, THREE);
   motor4->initialize(COUNTER_CLOCKWISE, FOUR);

   // attach the motors to the controller so that it can adjust speeds for stability and control
   controller->attachMotors(motor1, motor2, motor3, motor4);

   // initialize orientationManager
   orientationManager->initialize(imu, altitudeSensor);

// if not testing then finish initialization
#ifndef TESTING

   // wait for start command
   waitForStartCommand();

   // start up the quadrotor motors
   float startingSpeed = 10.0;
   startupSequence(startingSpeed);

   // start keeping track of the timing
   previousTime = millis();

   // last task is to setup the orientation manager with the orientation of the craft before takeoff
   orientationManager->initialRead(orientation);

   // initialize the control system with the initial data
   controller->initialize(orientation, startingSpeed);
#endif
}

// main program loop
void Quadrotor::loop()
{ 
#ifdef TESTING    // For testing
   // go into calibration mode for the bluetooth
   //bluetooth->calibrateLoop();
 //  orientationManager->initialRead(orientation);
	//for(;;) 
 //  {
 //     //Serial.println(altitudeSensor->getDistanceInches());
 //     deltaTime = 15/1000.0;
 //     orientationManager->getCurrentOrientation(deltaTime, orientation);
 //     Serial.print("Pitch: ");
 //     Serial.println(orientation.pitch);
 //     Serial.print("Roll: ");
 //     Serial.println(orientation.roll);
 //     Serial.print("Yaw: ");
 //     Serial.println(orientation.yaw);
 //     Serial.println("\n");
 //     delay(15);
 //  }

   struct Motors motors(motor1, motor2, motor3, motor4);
   Test test(&motors, orientationManager, controller, bluetooth, (ITG3200*)imu->getGyroscope(), (ADXL345*)imu->getAccelerometer(), (HMC5883L*)imu->getMagnetometer());
   //test.debug();
   //test.RPYTest();
   //test.PitchAxis();
   //test.RollPitchTest();
   test.FullFlightTest();
   test.AltitudeTest();
   test.yawOnStandTest();
   //test.RollPitchYawTest();
   //test.PitchAxisTest();  
   //test.PitchAxisTestWithMotorValues(); 
#endif            // end testing block

   currentTime = millis();
   if (currentTime - previousTime >= SAMPLE_RATE)
   {
      // get the delta time in seconds for calculations
      deltaTime = (currentTime - previousTime) / 1000.0;

      // get current orientation
      orientationManager->getCurrentOrientation(deltaTime, orientation);

      // use the current orientation to adjust the motors
      controller->flightControl(orientation, deltaTime);

      // check for commands via bluetooth
      message = bluetooth->readLine();

      // remove any whitespace
      message.trim();

      // handle the message if present
      if (!message.equals("")) handleMessage(message);

      previousTime = currentTime;
   }

   // wait for 30ms to complete
}

void Quadrotor::waitForStartCommand()
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

void Quadrotor::handleMessage(String message)
{
   if (message.equals("Escape"))
   {
      Serial.println("Handled an escape");
      motor1->setSpeed(0.0);
      motor2->setSpeed(0.0);
      motor3->setSpeed(0.0);
      motor4->setSpeed(0.0);

      for (;;); // spin forever
   }
}

void Quadrotor::shutdownSequence()
{
   unsigned long currentTime;
   unsigned long previousTime = millis();
   float deltaTime;
   struct RPYData orientation;
   String message;
   int count = 0;
   // slowly decrease the height
   int height = controller->getAltitudeController()->getSetpoint();
   while (height > 1)
   {
      currentTime = millis();
      if (currentTime - previousTime >= SAMPLE_RATE)
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
			   controller->flightControl(orientation, deltaTime);
			   // check for commands via bluetooth
			   message = bluetooth->readLine();
            // remove any whitespace
			   message.trim();

			   // handle the message if present
			   if (!message.equals("")) handleMessage(message);

            // every half a second lower the quadrotor an inch
			   if (count >= (int)floor(0.5 * 1000.0 / SAMPLE_RATE))
			   {
               count = 0;
               controller->getAltitudeController()->setSetpoint(--height);
			   }
		   }
      }
      // wait for 30ms to complete
   }
   // shut off motors
   motor1->setSpeed(0.0);
   motor2->setSpeed(0.0);
   motor3->setSpeed(0.0);
   motor4->setSpeed(0.0);
}

void Quadrotor::startupSequence(float startingSpeed)
{
   motor1->setSpeed(startingSpeed);
   motor2->setSpeed(startingSpeed);
   motor3->setSpeed(startingSpeed);
   motor4->setSpeed(startingSpeed);

   // wait for 1.5 seconds to allow motors to all turn on
   delay(1500);
}
