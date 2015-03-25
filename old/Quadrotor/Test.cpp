// 
// 
// 

#include "Test.h"
#include "QuadrotorAPI.h"

//const char *MATLAB_STOP_CONDITION = "StopTransmission";

Test::Test(struct Motors *motors, OrientationManager *orientationManager, ControllerManager *controllerManager, Bluetooth *bluetooth, ITG3200 *gyro, ADXL345 *accel, HMC5883L *mag)
   : motors(motors), orientationManager(orientationManager), controllerManager(controllerManager), bluetooth(bluetooth), gyro(gyro), accel(accel), mag(mag)
{

}

void Test::RPYTest()
{
   struct RPYData orientation;
   unsigned long previousTime;
   unsigned long currentTime;
   float deltaTime;
   String message;
   const int RATE_200HZ = 15;

   // char buffer stuff
   char buffer[100];
   // Arduino's representation of sprintf does not allow floating points
   // need to convert to char * and then can pass in as a string
   char roll[10];
   char pitch[10];
   char yaw[10];

   // wait for start signal
   waitForStartCommand();

   // start keeping track of the timing
   previousTime = millis();

   // last task is to setup the orientation manager with the orientation of the craft before takeoff
   orientationManager->initialRead(orientation);

   // test loop
   for(;;)
   {
		currentTime = millis();
		if (currentTime - previousTime >= RATE_200HZ)
		{
			// get the delta time in seconds for calculations
			deltaTime = (currentTime - previousTime) / 1000.0;

			// get current orientation
			orientationManager->getCurrentOrientation(deltaTime, orientation);

			// check for commands via bluetooth
			message = bluetooth->readLine();

			// remove any whitespace
			message.trim();

			// handle the message if present
			if (!message.equals("")) handleMessage(message);

			// send the current data
			dtostrf(orientation.roll, 8, 2, roll);
			dtostrf(orientation.pitch, 8, 2, pitch);
			dtostrf(orientation.yaw, 8, 2, yaw);
			sprintf(buffer, "Roll: %s Pitch: %s Yaw: %s", roll, pitch, yaw);
			bluetooth->writeLine(buffer);
			previousTime = currentTime;
      }
      // wait for 15ms to complete
   }
}

void Test::RollTest()
{
   struct RPYData filteredData;
   struct RPYData accelData;
   struct RPYData gyroData;
   float magYaw;

   RETURN_CODE retVal;
   int count = 0;

   unsigned long rate100Hz = 10;
   unsigned long previous = 0;
   unsigned long current;

   orientationManager->setGyroWeight(0.93);

   while (count < 200)
   {
      current = millis();
      if ( current - previous >= rate100Hz)
      {
         previous = current;
         if (orientationManager->getCurrentOrientation(0.01, filteredData, accelData, gyroData, magYaw) == SUCCESS)
         {
            char buffer[40];
            // Arduino's representation of sprintf does not allow floating points
            // need to convert to char * and then can pass in as a string
            char ar[10];
            dtostrf(accelData.roll, 8, 2, ar);
            char gr[10];
            dtostrf(gyroData.roll, 8, 2, gr);
            char fr[10];
            dtostrf(filteredData.roll, 8, 2, fr);
            sprintf(buffer, "Data: %s, %s, %s", ar, gr, fr);
            Serial.println(buffer);

            count++;
         }
         else
         {
            Serial.println("Error in test!!!");
            break;
         }
      }
   }

//   Serial.println(MATLAB_STOP_CONDITION);
}

void Test::YawTest()
{
   struct RPYData filteredData;
   struct RPYData accelData;
   struct RPYData gyroData;
   float magYaw;

   RETURN_CODE retVal;
   int count = 0;

   unsigned long rate100Hz = 10;
   unsigned long previous = 0;
   unsigned long current;

   orientationManager->setGyroWeight(1.0);

   while (true)
   //while (count < 1000)
   {
      current = millis();
      if ( current - previous >= rate100Hz)
      {
         previous = current;
         if (orientationManager->getCurrentOrientation(0.01, filteredData, accelData, gyroData, magYaw) == SUCCESS)
         {
            char buffer[40];
            // Arduino's representation of sprintf does not allow floating points
            // need to convert to char * and then can pass in as a string
            char my[10];
            dtostrf(magYaw, 8, 2, my);
            char gy[10];
            dtostrf(gyroData.yaw, 8, 2, gy);
            char fy[10];
            dtostrf(filteredData.yaw, 8, 2, fy);
            sprintf(buffer, "Data: %s, %s, %s", my, gy, fy);
            Serial.println(buffer);

            count++;
         }
         else
         {
            Serial.println("Error in test!!!");
            break;
         }
      }
   }

//   Serial.println(MATLAB_STOP_CONDITION);
}

void Test::GyroTest()
{
   struct XYZData data;
   float pitch;
   float roll;
   float yaw;

   gyro->setDLPFConfiguration(BW_256HZ);
   gyro->setSampleRateDivider(0x07);

   char buffer[40];
   char gr[10];
   char gp[10];
   char gy[10];

   accel->readData(data);

   float prevPitch = orientationManager->calculateAccelerometerPitch(data);
   float prevRoll = orientationManager->calculateAccelerometerRoll(data);;
   float prevYaw = 180;

   unsigned long nextRead100Hz = millis() + 10;
   unsigned long rate100Hz = 10;
   unsigned long previous = 0;
   unsigned long current;

   while (true)
   {
      current = millis();
      if ( (current - previous) >= rate100Hz)
      {
         previous = current;
         nextRead100Hz += 10;
         gyro->readData(data);

         pitch = orientationManager->calculateGyroscopePitch(data, prevPitch, 0.010);
         roll = orientationManager->calculateGyroscopeRoll(data, prevRoll, 0.010);
         yaw = orientationManager->calculateGyroscopeYaw(data, prevYaw, 0.010);
         // Arduino's representation of sprintf does not allow floating points
         // need to convert to char * and then can pass in as a string

         dtostrf(roll, 8, 2, gr);
         dtostrf(pitch, 8, 2, gp);
         dtostrf(yaw, 8, 2, gy);
         sprintf(buffer, "Roll: %s, Pitch: %s, Yaw: %s", gr, gp, gy);
         Serial.println(buffer);

         prevYaw = yaw;
         prevRoll = roll;
         prevPitch = pitch;
      }
   }
}

void Test::AccelTest()
{
   struct XYZData data;
   float pitch;
   float roll;

   unsigned long nextRead100Hz = millis() + 10;

   accel->setRange(RANGE_2G);
   accel->setSampleRate(NORMAL_POWER, RATE_200HZ);

   char buffer[30];
   char ar[10];
   char ap[10];

   while (true)
   {
      if ( millis() >= nextRead100Hz)
      {
         nextRead100Hz += 10;
         accel->readData(data);

         pitch = orientationManager->calculateAccelerometerPitch(data);
         roll = orientationManager->calculateAccelerometerRoll(data);

         // Arduino's representation of sprintf does not allow floating points
         // need to convert to char * and then can pass in as a string

         dtostrf(roll, 8, 2, ar);
         dtostrf(pitch, 8, 2, ap);
         sprintf(buffer, "Roll: %s, Pitch: %s", ar, ap);
         Serial.println(buffer);
      }
   }
}

void Test::MagTest()
{
   struct XYZData data;
   float tiltYaw;
   float untiltedYaw;

   unsigned long nextRead100Hz = millis() + 10;

   mag->setRange(GAUSS_1_3);

   char buffer[30];
   char my[10];
   char mty[10];

   while (true)
   {
      if ( millis() >= nextRead100Hz)
      {
         nextRead100Hz += 10;
         mag->readData(data);


         // Arduino's representation of sprintf does not allow floating points
         // need to convert to char * and then can pass in as a string

         dtostrf(tiltYaw, 8, 2, mty);
         dtostrf(untiltedYaw, 8, 2, my);
         sprintf(buffer, "TiltComp: %s, Raw: %s", mty, my);
         Serial.println(buffer);
      }
   }
}

void Test::PitchAxisTest()
{
   struct RPYData orientation;
   unsigned long previousTime;
   unsigned long currentTime;
   float deltaTime;
   String message;
   const int RATE_100HZ = 10;

   // char buffer stuff
   char buffer[100];
   // Arduino's representation of sprintf does not allow floating points
   // need to convert to char * and then can pass in as a string
   char kP[10];
   char kI[10];
   char kD[10];
   char pitch[10];
   char set[10];
   char cnt[10];

   // test values
   float kPvalue = 0.3;
   float kIvalue = 0.0;
   float kDvalue = 0.0;
   float setpoint = 0.0;

   // convert floats to strings
   dtostrf(kPvalue, 8, 5, kP);
   dtostrf(kIvalue, 8, 5, kI);
   dtostrf(kDvalue, 8, 5, kD);
   sprintf(buffer, "PID values: %s, %s, %s", kP, kI, kD);

   // start keeping track of the timing
   previousTime = millis();

   // last task is to setup the orientation manager with the orientation of the craft before takeoff
   orientationManager->initialRead(orientation);

   // initialize the control system with the initial data
   controllerManager->initialize(orientation, 0.0);

   // setup the pitch controller for the test - overwrite whatever ControllerManager::initialize() setup
   controllerManager->getPitchController()->setConstants(kPvalue, kIvalue, kDvalue);
   controllerManager->getPitchController()->setSetpoint(setpoint);

   // send setup data
   bluetooth->writeLine("Pitch Axis Test");
   bluetooth->writeLine("Count: %f Pitch: %f Setpoint: %f");
   bluetooth->writeLine(buffer);

   // turn on motors
   motors->m1->setSpeed(30.0);
   motors->m3->setSpeed(30.0);

   // test loop
   const unsigned long iterations = 800;
   for(unsigned long count = 0; count < iterations; )
   {
      currentTime = millis();
      if (currentTime - previousTime >= RATE_100HZ)
      {
         ++count;
         // get the delta time in seconds for calculations
         deltaTime = (currentTime - previousTime) / 1000.0;

         // get current orientation
         orientationManager->getCurrentOrientation(deltaTime, orientation);

         // use the current orientation to adjust the motors
         controllerManager->flightControl(orientation, deltaTime);

         // check for commands via bluetooth
         message = bluetooth->readLine();

         // remove any whitespace
         message.trim();

         // handle the message if present
         if (!message.equals("")) handleMessage(message);

         // send the current data
         dtostrf(count, 8, 0, cnt); // don't know why it is being dumb and can't just take an int
         dtostrf(orientation.pitch, 8, 2, pitch);
         dtostrf(setpoint, 8, 2, set);
         sprintf(buffer, "Count: %s Pitch: %s Setpoint: %s", cnt, pitch, set);
         bluetooth->writeLine(buffer);

         previousTime = currentTime;
      }
      // wait for 10ms to complete
   }
   motors->m1->setSpeed(0.0);
   motors->m2->setSpeed(0.0);
   motors->m3->setSpeed(0.0);
   motors->m4->setSpeed(0.0);

   for (;;);  
}

void Test::PitchAxisTestWithMotorValues()
{
   struct ControllerOutput output;
   struct RPYData orientation;
   unsigned long previousTime;
   unsigned long currentTime;
   float deltaTime;
   String message;
   const int RATE_200HZ = 15;

   // char buffer stuff
   char buffer[100];
   // Arduino's representation of sprintf does not allow floating points
   // need to convert to char * and then can pass in as a string
   char kP[10];
   char kI[10];
   char kD[10];
   char pitch[10];
   char set[10];
   char cnt[10];
   char m1v[10];
   char m3v[10];
   char pid[10];

   // test values
   float kPvalue = 0.01;
   float kIvalue = 0;
   float kDvalue = 0;
   float setpoint = 0.0;

   // convert floats to strings
   dtostrf(kPvalue, 8, 5, kP);
   dtostrf(kIvalue, 8, 5, kI);
   dtostrf(kDvalue, 8, 5, kD);
   sprintf(buffer, "PID values: %s, %s, %s", kP, kI, kD);

   // start keeping track of the timing
   previousTime = millis();

   // last task is to setup the orientation manager with the orientation of the craft before takeoff
   orientationManager->initialRead(orientation);

   // initialize the control system with the initial data
   controllerManager->initialize(orientation, 0.0);

   // setup the pitch controller for the test - overwrite whatever ControllerManager::initialize() setup
   controllerManager->getPitchController()->setConstants(kPvalue, kIvalue, kDvalue);
   controllerManager->getPitchController()->setSetpoint(setpoint);

   // send setup data
   bluetooth->writeLine("Pitch Axis Test With Motor Values");
   bluetooth->writeLine("Count: %f Pitch: %f Setpoint: %f M1: %f M3: %f PID: %f");
   bluetooth->writeLine(buffer);

   // turn on motors
   motors->m1->setSpeed(10.0);
   motors->m3->setSpeed(10.0);

   // test loop
   const unsigned long iterations = 800;
   for(unsigned long count = 0; count < iterations; )
   {
      currentTime = millis();
      if (currentTime - previousTime >= RATE_200HZ)
      {
         ++count;
         // get the delta time in seconds for calculations
         deltaTime = (currentTime - previousTime) / 1000.0;

         // get current orientation
         orientationManager->getCurrentOrientation(deltaTime, orientation);

         // use the current orientation to adjust the motors
         controllerManager->flightControl(orientation, deltaTime, output);

         // check for commands via bluetooth
         message = bluetooth->readLine();

         // remove any whitespace
         message.trim();

         // handle the message if present
         if (!message.equals("")) handleMessage(message);

         // send the current data
         dtostrf(count, 8, 0, cnt); // don't know why it is being dumb and can't just take an int
         dtostrf(orientation.pitch, 8, 2, pitch);
         dtostrf(setpoint, 8, 2, set);
         dtostrf(motors->m1->getSpeed(), 8, 2, m1v);
         dtostrf(motors->m3->getSpeed(), 8, 2, m3v);
         dtostrf(output.pitch, 8, 2, pid);
         sprintf(buffer, "Count: %s Pitch: %s Setpoint: %s M1: %s M3: %s PID: %s", cnt, pitch, set, m1v, m3v, pid);
         bluetooth->writeLine(buffer);
         sprintf(buffer, "Pitch: %s",pitch);
         Serial.println(buffer);
         previousTime = currentTime;
         //Serial.println(millis() - currentTime);
      }
      // wait for 10ms to complete
   }
   motors->m1->setSpeed(0.0);
   motors->m2->setSpeed(0.0);
   motors->m3->setSpeed(0.0);
   motors->m4->setSpeed(0.0);

   for (;;);  
}

void Test::PitchAxis()
{
   // wait for start signal
   waitForStartCommand();

   struct TestParameters tests[] = 
   {
      // NOTE: test various kP values with these kI and kD - they seem pretty good
      //TestParameters(kP,   kI,   kD,  gyroWeight)
      TestParameters(  1.30, 0.4, 0.30, 0.98)// GOOD
	  //TestParameters(  1.3, 0.4, 0.15, 0.98),
	  //TestParameters(  1.3, 0.45, 0.15, 0.98),
	  //TestParameters(  1.3, 0.50, 0.15, 0.98),
	  //TestParameters(  1.3, 0.55, 0.15, 0.98)
	  
		// We have found that these: 1.30, 0.4, 0.30, 0.98 are the best parameters for the most stable flight
   };
   const unsigned long iterations = 1000; // 700 is about 10 seconds
   int numOfTests = sizeof(tests)/sizeof(struct TestParameters);

   // char buffer stuff
   char buffer[100];
   char num[10];
   // send the number of tests
   dtostrf(numOfTests, 8, 5, num);
   sprintf(buffer, "Tests: %s", num);
   bluetooth->writeLine(buffer);

   for(int i=0; i<numOfTests; ++i)
   {
      struct RPYData orientation;
      unsigned long previousTime;
      unsigned long currentTime;
      float deltaTime;
      String message;
      const int RATE_200HZ = 15;

      // test values
      float setpoint = 0.0;

      // Arduino's representation of sprintf does not allow floating points
      // need to convert to char * and then can pass in as a string
      char kP[10];
      char kI[10];
      char kD[10];
      char gWeight[10];
      char pitch[10];
      char set[10];
      char cnt[10];
      char m1v[10];
      char m3v[10];

      // wait for start signal
      waitForStartCommand();

      // convert floats to strings
      dtostrf(tests[i].kP, 8, 5, kP);
      dtostrf(tests[i].kI, 8, 5, kI);
      dtostrf(tests[i].kD, 8, 5, kD);
      dtostrf(tests[i].gyroWeight, 8, 5, gWeight); 
      sprintf(buffer, "PID values: %s, %s, %s Gyro Weight: %s", kP, kI, kD, gWeight);

      // start keeping track of the timing
      previousTime = millis();

      // last task is to setup the orientation manager with the orientation of the craft before takeoff
      orientationManager->initialRead(orientation);

      // initialize the control system with the initial data
      controllerManager->initialize(orientation, 0.0);

      // setup the pitch controller for the test - overwrite whatever ControllerManager::initialize() setup
      controllerManager->getPitchController()->setConstants(tests[i].kP, tests[i].kI, tests[i].kD);
      controllerManager->getPitchController()->setSetpoint(setpoint);
      orientationManager->setGyroWeight(tests[i].gyroWeight);

      // send setup data
      bluetooth->writeLine("Pitch Axis Test"); // name of test
      bluetooth->writeLine("Count: %f Pitch: %f Setpoint: %f M1: %f M3: %f"); // expected format of the data - for Matlab
      bluetooth->writeLine(buffer); // test parameters

      // test loop
      for(unsigned long count = 0; count < iterations; )
      {
         currentTime = millis();
         if (currentTime - previousTime >= RATE_200HZ)
         {
            ++count;
            // get the delta time in seconds for calculations
            deltaTime = (currentTime - previousTime) / 1000.0;

            // get current orientation
            orientationManager->getCurrentOrientation(deltaTime, orientation);

            // use the current orientation to adjust the motors
            controllerManager->flightControl(orientation, deltaTime);

            // check for commands via bluetooth
            message = bluetooth->readLine();

            // remove any whitespace
            message.trim();

            // handle the message if present
            if (!message.equals("")) handleMessage(message);

            // send the current data
            dtostrf(count, 8, 0, cnt); // don't know why it is being dumb and can't just take an int
            dtostrf(orientation.pitch, 8, 2, pitch);
            dtostrf(setpoint, 8, 2, set);
            dtostrf(motors->m1->getSpeed(), 8, 2, m1v);
            dtostrf(motors->m3->getSpeed(), 8, 2, m3v);
            sprintf(buffer, "Count: %s Pitch: %s Setpoint: %s M1: %s M3: %s", cnt, pitch, set, m1v, m3v);
            bluetooth->writeLine(buffer);
            sprintf(buffer, "Pitch: %s",pitch);
            Serial.println(buffer);
            previousTime = currentTime;
            //Serial.println(millis() - currentTime);
         }
         // wait for 15ms to complete
      }
      // completion of test i
      motors->m1->setSpeed(0.0);
      motors->m2->setSpeed(0.0);
      motors->m3->setSpeed(0.0);
      motors->m4->setSpeed(0.0);

      bluetooth->writeLine("Complete Test");
   }

   bluetooth->writeLine("Done");

   // tests complete
   for (;;);  
}

void Test::RollPitchTest()
{
   // wait for start signal
   waitForStartCommand();

   // set altitude controller parameters
   controllerManager->getAltitudeController()->setConstants(1.0, 0.1, 0.1); // kP, kI, kD
   controllerManager->getAltitudeController()->setSetpoint(12.0); // in inches

   struct TestParameters rollParams[] = 
   {
      // NOTE: test various kP values with these kI and kD - they seem pretty good
      //TestParameters(kP,   kI,   kD,  gyroWeight)
      TestParameters(  1.30, 0.4, 0.30, 0.98)
	  //TestParameters(  1.3, 0.4, 0.15, 0.98),
	  //TestParameters(  1.3, 0.45, 0.15, 0.98),
	  //TestParameters(  1.3, 0.50, 0.15, 0.98),
	  //TestParameters(  1.3, 0.55, 0.15, 0.98)
	  
   };
   struct TestParameters pitchParams[] = 
   {
      // NOTE: test various kP values with these kI and kD - they seem pretty good
      //TestParameters(kP,   kI,   kD,  gyroWeight)
      TestParameters(  1.30, 0.4, 0.30, 0.98)// GOOD
	  //TestParameters(  1.3, 0.4, 0.15, 0.98),
	  //TestParameters(  1.3, 0.45, 0.15, 0.98),
	  //TestParameters(  1.3, 0.50, 0.15, 0.98),
	  //TestParameters(  1.3, 0.55, 0.15, 0.98)
	  
		// We have found that these: 1.30, 0.4, 0.30, 0.98 are the best parameters for the most stable flight
   };
   const unsigned long iterations = 1000; // 700 is about 10 seconds
   int numOfTests = sizeof(rollParams)/sizeof(struct TestParameters);

   // char buffer stuff
   char buffer[100];
   char num[10];
   // send the number of tests
   dtostrf(numOfTests, 8, 5, num);
   sprintf(buffer, "Tests: %s", num);
   bluetooth->writeLine(buffer);

   for(int i=0; i<numOfTests; ++i)
   {
      struct RPYData orientation;
      unsigned long previousTime;
      unsigned long currentTime;
      float deltaTime;
      String message;
      const int RATE_200HZ = 15;

      // test values
      float setpoint = 0.0;

      // Arduino's representation of sprintf does not allow floating points
      // need to convert to char * and then can pass in as a string
      char rkP[10];
      char rkI[10];
      char rkD[10];
      char pkP[10];
      char pkI[10];
      char pkD[10];
      char roll[10];
      char pitch[10];
      char set[10];
      char cnt[10];

      // wait for start signal
      waitForStartCommand();

      // convert floats to strings
      dtostrf(rollParams[i].kP, 8, 5, rkP);
      dtostrf(rollParams[i].kI, 8, 5, rkI);
      dtostrf(rollParams[i].kD, 8, 5, rkD);
      dtostrf(pitchParams[i].kP, 8, 5, pkP);
      dtostrf(pitchParams[i].kI, 8, 5, pkI);
      dtostrf(pitchParams[i].kD, 8, 5, pkD);
      sprintf(buffer, "Roll PID values: %s, %s, %s Pitch PID values: %s, %s, %s", rkP, rkI, rkD, pkP, pkI, pkD);

      // start keeping track of the timing
      previousTime = millis();

      // last task is to setup the orientation manager with the orientation of the craft before takeoff
      orientationManager->initialRead(orientation);

      // initialize the control system with the initial data
      controllerManager->initialize(orientation, 0.0);

      // setup the roll controller for the test - overwrite whatever ControllerManager::initialize() setup
      controllerManager->getRollController()->setConstants(rollParams[i].kP, rollParams[i].kI, rollParams[i].kD);
      controllerManager->getRollController()->setSetpoint(setpoint);
      // setup the pitch controller for the test - overwrite whatever ControllerManager::initialize() setup
      controllerManager->getPitchController()->setConstants(pitchParams[i].kP, pitchParams[i].kI, pitchParams[i].kD);
      controllerManager->getPitchController()->setSetpoint(setpoint);
      orientationManager->setGyroWeight(rollParams[i].gyroWeight);

      // send setup data
      bluetooth->writeLine("Roll/Pitch Test"); // name of test
      bluetooth->writeLine("Count: %f Roll: %f Pitch: %f Setpoint: %f"); // expected format of the data - for Matlab
      bluetooth->writeLine(buffer); // test parameters

      // test loop
      for(unsigned long count = 0; count < iterations; )
      {
         currentTime = millis();
         if (currentTime - previousTime >= RATE_200HZ)
         {
            ++count;
            // get the delta time in seconds for calculations
            deltaTime = (currentTime - previousTime) / 1000.0;

            // get current orientation
            orientationManager->getCurrentOrientation(deltaTime, orientation);

            // use the current orientation to adjust the motors
            controllerManager->flightControl(orientation, deltaTime);

            // check for commands via bluetooth
            message = bluetooth->readLine();

            // remove any whitespace
            message.trim();

            // handle the message if present
            if (!message.equals("")) handleMessage(message);

            // send the current data
            dtostrf(count, 8, 0, cnt);
            dtostrf(orientation.roll, 8, 2, roll);
            dtostrf(orientation.pitch, 8, 2, pitch);
            dtostrf(setpoint, 8, 2, set);
            sprintf(buffer, "Count: %s Roll: %s Pitch: %s Setpoint: %s", cnt, roll, pitch, set);
            bluetooth->writeLine(buffer);
            sprintf(buffer, "Roll: %s Pitch: %s",roll, pitch);
            Serial.println(buffer);
            previousTime = currentTime;
            //Serial.println(millis() - currentTime);
         }
         // wait for 15ms to complete
      }
      // completion of test i
      motors->m1->setSpeed(0.0);
      motors->m2->setSpeed(0.0);
      motors->m3->setSpeed(0.0);
      motors->m4->setSpeed(0.0);

      bluetooth->writeLine("Complete Test");
   }

   bluetooth->writeLine("Done");

   // tests complete
   for (;;);
}

void Test::RollPitchYawTest()
{
   // wait for start signal
   waitForStartCommand();

   // set altitude controller parameters
   controllerManager->getAltitudeController()->setConstants(1.0, 0.1, 0.1); // kP, kI, kD
   controllerManager->getAltitudeController()->setSetpoint(12.0); // in inches

   struct TestParameters rollParams[] = 
   {
      // NOTE: test various kP values with these kI and kD - they seem pretty good
      //TestParameters(kP,   kI,   kD, gyroWeight)
      TestParameters(  1.30, 0.4, 0.30, 0.98)
	  //TestParameters(  1.3, 0.4, 0.15, 0.98),
	  //TestParameters(  1.3, 0.45, 0.15, 0.98),
	  //TestParameters(  1.3, 0.50, 0.15, 0.98),
	  //TestParameters(  1.3, 0.55, 0.15, 0.98)
	  
   };
   struct TestParameters pitchParams[] = 
   {
      // NOTE: test various kP values with these kI and kD - they seem pretty good
      //TestParameters(kP,   kI,   kD,  gyroWeight)
      TestParameters(  1.30, 0.4, 0.30, 0.98)// GOOD
	  //TestParameters(  1.3, 0.4, 0.15, 0.98),
	  //TestParameters(  1.3, 0.45, 0.15, 0.98),
	  //TestParameters(  1.3, 0.50, 0.15, 0.98),
	  //TestParameters(  1.3, 0.55, 0.15, 0.98)
	  
		// We have found that these: 1.30, 0.4, 0.30, 0.98 are the best parameters for the most stable flight
   };
   const unsigned long iterations = 1000; // 700 is about 10 seconds
   int numOfTests = sizeof(rollParams)/sizeof(struct TestParameters);

   // char buffer stuff
   char buffer[100];
   char num[10];
   // send the number of tests
   dtostrf(numOfTests, 8, 5, num);
   sprintf(buffer, "Tests: %s", num);
   bluetooth->writeLine(buffer);

   for(int i=0; i<numOfTests; ++i)
   {
      struct RPYData orientation;
      unsigned long previousTime;
      unsigned long currentTime;
      float deltaTime;
      String message;
      const int RATE_200HZ = 15;

      // test values
      float setpoint = 0.0;

      // Arduino's representation of sprintf does not allow floating points
      // need to convert to char * and then can pass in as a string
      char rkP[10];
      char rkI[10];
      char rkD[10];
      char pkP[10];
      char pkI[10];
      char pkD[10];
      char roll[10];
      char pitch[10];
      char set[10];
      char cnt[10];

      // wait for start signal
      waitForStartCommand();
            
      // convert floats to strings
      dtostrf(rollParams[i].kP, 8, 5, rkP);
      dtostrf(rollParams[i].kI, 8, 5, rkI);
      dtostrf(rollParams[i].kD, 8, 5, rkD);
      dtostrf(pitchParams[i].kP, 8, 5, pkP);
      dtostrf(pitchParams[i].kI, 8, 5, pkI);
      dtostrf(pitchParams[i].kD, 8, 5, pkD);
      sprintf(buffer, "Roll PID values: %s, %s, %s Pitch PID values: %s, %s, %s", rkP, rkI, rkD, pkP, pkI, pkD);

      // start keeping track of the timing
      previousTime = millis();

      // last task is to setup the orientation manager with the orientation of the craft before takeoff
      orientationManager->initialRead(orientation);

      // initialize the control system with the initial data
      controllerManager->initialize(orientation, 0.0);

      // setup the roll controller for the test - overwrite whatever ControllerManager::initialize() setup
      controllerManager->getRollController()->setConstants(rollParams[i].kP, rollParams[i].kI, rollParams[i].kD);
      controllerManager->getRollController()->setSetpoint(setpoint);
      // setup the pitch controller for the test - overwrite whatever ControllerManager::initialize() setup
      controllerManager->getPitchController()->setConstants(pitchParams[i].kP, pitchParams[i].kI, pitchParams[i].kD);
      controllerManager->getPitchController()->setSetpoint(setpoint);
      orientationManager->setGyroWeight(rollParams[i].gyroWeight);

      // send setup data
      bluetooth->writeLine("Roll/Pitch Test"); // name of test
      bluetooth->writeLine("Count: %f Roll: %f Pitch: %f Setpoint: %f"); // expected format of the data - for Matlab
      bluetooth->writeLine(buffer); // test parameters

      startupSequence();

      delay(2000);

      // test loop
      for(unsigned long count = 0; count < iterations; )
      {
         currentTime = millis();
         if (currentTime - previousTime >= RATE_200HZ)
         {
            ++count;
            // get the delta time in seconds for calculations
            deltaTime = (currentTime - previousTime) / 1000.0;

            // get current orientation
            orientationManager->getCurrentOrientation(deltaTime, orientation);

            // use the current orientation to adjust the motors
            controllerManager->flightControl(orientation, deltaTime);

            // check for commands via bluetooth
            message = bluetooth->readLine();

            // remove any whitespace
            message.trim();

            // handle the message if present
            if (!message.equals("")) handleMessage(message);

            // send the current data
            dtostrf(count, 8, 0, cnt);
            dtostrf(orientation.roll, 8, 2, roll);
            dtostrf(orientation.pitch, 8, 2, pitch);
            dtostrf(setpoint, 8, 2, set);
            sprintf(buffer, "Count: %s Roll: %s Pitch: %s Setpoint: %s", cnt, roll, pitch, set);
            bluetooth->writeLine(buffer);
            sprintf(buffer, "Roll: %s Pitch: %s",roll, pitch);
            Serial.println(buffer);
            previousTime = currentTime;
            //Serial.println(millis() - currentTime);
         }
         // wait for 15ms to complete
      }
      // completion of test i
      motors->m1->setSpeed(0.0);
      motors->m2->setSpeed(0.0);
      motors->m3->setSpeed(0.0);
      motors->m4->setSpeed(0.0);

      bluetooth->writeLine("Complete Test");
   }

   bluetooth->writeLine("Done");

   // tests complete
   for (;;);
}

void Test::yawOnStandTest()
{
   // wait for start signal
   waitForStartCommand();

   struct TestParameters rollValues (1.4, 0.4, 0.3,          /*not needed*/ 0); 
   struct TestParameters pitchValues(1.4, 0.4, 0.3,          /*not needed*/ 0);
   struct TestParameters altitudeValues(0.03, 0.01, 0.005,   /*not needed*/ 0.98);

   //struct TestParameters altParams[] = 
   //{
   //   // NOTE: test various kP values with these kI and kD - they seem pretty good
   //   //TestParameters(kP,   kI,   kD,   gyroWeight)
   //   //TestParameters(  1.00, 0.10, 0.10, 0.98)
	  ////TestParameters(  0.04, 0, 0, 0.98),
	  //TestParameters(  0.03, 0.01, 0.005, 0.98)
   //};

   struct TestParameters yawParameters[] = 
   {
      //TestParameters(kP,   kI,   kD,   gyroWeight)
      TestParameters(1.4, 0.4, 0.3, .98)
      //TestParameters(1.4, 0.4, 0.35, .98),
      //TestParameters(1.4, 0.4, 0.4, .98),
      //TestParameters(1.4, 0.4, 0.45, .98),
      //TestParameters(1.4, 0.4, 0.5, .98)
   };


   float altSetpoint = 13.0;

   const unsigned long iterations = 800; // 700 is about 10 seconds
   int numOfTests = sizeof(yawParameters)/sizeof(struct TestParameters);

   // char buffer stuff
   char buffer[100];
   char num[10];
   // send the number of tests
   dtostrf(numOfTests, 8, 5, num);
   sprintf(buffer, "Tests: %s", num);
   bluetooth->writeLine(buffer);

   for(int i=0; i<numOfTests; ++i)
   { 
      struct RPYData orientation;
      unsigned long previousTime;
      unsigned long currentTime;
      float deltaTime;
      String message;
      const int RATE_200HZ = 30;

      // Arduino's representation of sprintf does not allow floating points
      // need to convert to char * and then can pass in as a string
      char akP[10];
      char akI[10];
      char akD[10];
      char altSet[10];
	  char alt[10];
      char cnt[10];

      // wait for start signal
      waitForStartCommand();

      startupSequence();
      delay(400);

      // convert floats to strings
      dtostrf(yawParameters[i].kP, 8, 5, akP);
      dtostrf(yawParameters[i].kI, 8, 5, akI);
      dtostrf(yawParameters[i].kD, 8, 5, akD);
      sprintf(buffer, "PID values: %s, %s, %s", akP, akI, akD);

      // start keeping track of the timing
      previousTime = millis();

      // last task is to setup the orientation manager with the orientation of the craft before takeoff
      orientationManager->initialRead(orientation);

      // initialize the control system with the initial data
      controllerManager->initialize(orientation, 25.0);

      // setup the roll controller for the test - overwrite whatever ControllerManager::initialize() setup
      controllerManager->getRollController()->setConstants(rollValues.kP, rollValues.kI, rollValues.kD);
      controllerManager->getRollController()->setSetpoint(0.0);
      // setup the pitch controller for the test - overwrite whatever ControllerManager::initialize() setup
      controllerManager->getPitchController()->setConstants(pitchValues.kP, pitchValues.kI, pitchValues.kD);
      controllerManager->getPitchController()->setSetpoint(0.0);
      // setup the yaw controller for the test - overwrite whatever ControllerManager::initialize() setup
      controllerManager->getYawController()->setConstants(yawParameters[i].kP, yawParameters[i].kI, yawParameters[i].kD);
      controllerManager->getYawController()->setSetpoint(180.0);
      // setup the altitude controller for the test - overwrite whatever ControllerManager::initialize() setup
      controllerManager->getAltitudeController()->setConstants(altitudeValues.kP, altitudeValues.kI, altitudeValues.kD);
      controllerManager->getAltitudeController()->setSetpoint(altSetpoint);

      orientationManager->setGyroWeight(yawParameters[i].gyroWeight);

      // send setup data
      bluetooth->writeLine("Altitude Test"); // name of test
      bluetooth->writeLine("Count: %f Altitude: %f Setpoint: %f"); // expected format of the data - for Matlab
      bluetooth->writeLine(buffer); // test parameters



      // test loop
      for(unsigned long count = 0; count < iterations; )
      {
         currentTime = millis();
         if (currentTime - previousTime >= RATE_200HZ)
         {
            ++count;
			
            // get the delta time in seconds for calculations
            deltaTime = (currentTime - previousTime) / 1000.0;

			   previousTime = currentTime;

            // get current orientation
            RETURN_CODE retVal = orientationManager->getCurrentOrientation(deltaTime, orientation);
			   if (retVal == SUCCESS)
			   {
               
               Serial1.println(orientation.yaw);
               
               //Serial1.println(orientation.altitude);
				   
               // use the current orientation to adjust the motors
				   controllerManager->flightControl(orientation, deltaTime);
				   // check for commands via bluetooth
				   message = bluetooth->readLine();

				   // remove any whitespace
				   message.trim();

				   // handle the message if present
				   if (!message.equals("")) handleMessage(message);

				   // send the current data
				   //dtostrf(count, 8, 0, cnt);
				   //dtostrf(orientation.altitude, 8, 2, alt);
				   //dtostrf(altSetpoint, 8, 2, altSet);
				   //sprintf(buffer, "Count: %s Altitude: %s Setpoint: %s", cnt, alt, altSet);
				   //bluetooth->writeLine(buffer);
            
				   //Serial.println(millis() - currentTime);
			   }
            else 
            {
               // error occured
               break;
            }
         }
         // wait for 15ms to complete
      }
      // completion of test i
      shutdownSequence();

      bluetooth->writeLine("Complete Test");
   }

   bluetooth->writeLine("Done");

   // tests complete
   for (;;);
}

void Test::FullFlightTest()
{
   // wait for start signal
   waitForStartCommand();

   struct TestParameters rollValues (1.6, 0.2, 0.6,      /*not needed*/ 0); 
   struct TestParameters pitchValues(1.6, 0.2, 0.6,     /*not needed*/ 0);  
   struct TestParameters yawValues  (1.4, 0.4, 0.3,      /*not needed*/ 0);

   struct TestParameters altParams[] = 
   {
      // NOTE: test various kP values with these kI and kD - they seem pretty good
      //TestParameters(kP,   kI,   kD,   gyroWeight)
      //TestParameters(  1.00, 0.10, 0.10, 0.98)
	  //TestParameters(  0.04, 0, 0, 0.98),
	  TestParameters(  0.05, 0.01, 0.01, 0.98)
   };
   float altSetpoint = 12.0;

   const unsigned long iterations = 200;
   int numOfTests = sizeof(altParams)/sizeof(struct TestParameters);

   // char buffer stuff
   char buffer[200];
   char num[10];
   // send the number of tests
   dtostrf(numOfTests, 8, 5, num);
   sprintf(buffer, "Tests: %s", num);
   bluetooth->writeLine(buffer);

   for(int i=0; i<numOfTests; ++i)
   { 
      struct RPYData orientation;
      unsigned long previousTime;
      unsigned long currentTime;
      float deltaTime;
      String message;
      const int SAMPLE_FREQ = 30;

      // Arduino's representation of sprintf does not allow floating points
      // need to convert to char * and then can pass in as a string
      char rkP[10];
      char rkI[10];
      char rkD[10];
      char pkP[10];
      char pkI[10];
      char pkD[10];
      char akP[10];
      char akI[10];
      char akD[10];
      char ykP[10];
      char ykI[10];
      char ykD[10];

      // wait for start signal
      waitForStartCommand();

      startupSequence();
      delay(1500);

      // convert floats to strings
      dtostrf(rollValues.kP, 8, 5, rkP);
      dtostrf(rollValues.kI, 8, 5, rkI);
      dtostrf(rollValues.kD, 8, 5, rkD);
      dtostrf(pitchValues.kP, 8, 5, pkP);
      dtostrf(pitchValues.kI, 8, 5, pkI);
      dtostrf(pitchValues.kD, 8, 5, pkD);
      dtostrf(altParams[i].kP, 8, 5, akP);
      dtostrf(altParams[i].kI, 8, 5, akI);
      dtostrf(altParams[i].kD, 8, 5, akD);
      dtostrf(yawValues.kP, 8, 5, ykP);
      dtostrf(yawValues.kI, 8, 5, ykI);
      dtostrf(yawValues.kD, 8, 5, ykD);
      sprintf(buffer, "PID values: Roll: %s, %s, %s Pitch: %s, %s, %s Yaw: %s, %s, %s Alt: %s, %s, %s", rkP, rkI, rkD, pkP, pkI, pkD, ykP, ykI, ykD, akP, akI, akD);

      // start keeping track of the timing
      previousTime = millis();

      // last task is to setup the orientation manager with the orientation of the craft before takeoff
      orientationManager->initialRead(orientation);

      // initialize the control system with the initial data
      controllerManager->initialize(orientation, 10.0);

      // setup the roll controller for the test - overwrite whatever ControllerManager::initialize() setup
      controllerManager->getRollController()->setConstants(rollValues.kP, rollValues.kI, rollValues.kD);
      controllerManager->getRollController()->setSetpoint(0.0);
      // setup the pitch controller for the test - overwrite whatever ControllerManager::initialize() setup
      controllerManager->getPitchController()->setConstants(pitchValues.kP, pitchValues.kI, pitchValues.kD);
      controllerManager->getPitchController()->setSetpoint(0.0);
      // setup the yaw controller for the test - overwrite whatever ControllerManager::initialize() setup
      controllerManager->getYawController()->setConstants(yawValues.kP, yawValues.kI, yawValues.kD);
      controllerManager->getYawController()->setSetpoint(180.0);
      // setup the altitude controller for the test - overwrite whatever ControllerManager::initialize() setup
      controllerManager->getAltitudeController()->setConstants(altParams[i].kP, altParams[i].kI, altParams[i].kD);
      controllerManager->getAltitudeController()->setSetpoint(altSetpoint);

      orientationManager->setGyroWeight(altParams[i].gyroWeight);

      // send setup data
      bluetooth->writeLine("Full Flight Test"); // name of test
      bluetooth->writeLine("Count: %f Roll: %f Pitch: %f Setpoint: %f Yaw: %f Setpoint: %f Altitude: %f Setpoint: %f"); // expected format of the data - for Matlab
      bluetooth->writeLine(buffer); // test parameters

      // turn on motors

      // test loop
      for(unsigned long count = 0; count < iterations; )
      {
         currentTime = millis();
         if (currentTime - previousTime >= SAMPLE_FREQ)
         {
            ++count;
			
            // get the delta time in seconds for calculations
            deltaTime = (currentTime - previousTime) / 1000.0;
			   previousTime = currentTime;

            // get current orientation
            RETURN_CODE retVal = orientationManager->getCurrentOrientation(deltaTime, orientation);
			   if (retVal == SUCCESS)
			   {
               // get current orientation
               orientationManager->getCurrentOrientation(deltaTime, orientation);

               // use the current orientation to adjust the motors
               controllerManager->flightControl(orientation, deltaTime);

               // check for commands via bluetooth
               message = bluetooth->readLine();

               // remove any whitespace
               message.trim();

               // handle the message if present
               if (!message.equals("")) handleMessage(message);

               Serial1.print("Count: ");     Serial1.print(count);
               Serial1.print(" Roll: ");     Serial1.print(orientation.roll);
               Serial1.print(" Pitch: ");    Serial1.print(orientation.pitch);
               Serial1.print(" Setpoint: "); Serial1.print(0.0);
               Serial1.print(" Yaw: ");      Serial1.print(orientation.yaw);
               Serial1.print(" Setpoint: "); Serial1.print(180.0);
               Serial1.print(" Altitude: "); Serial1.print(orientation.altitude);
               Serial1.print(" Setpoint: "); Serial1.println(altSetpoint);
            }
            //Serial.println(millis() - currentTime);
         }
         // wait for 30ms to complete
      }
      // completion of test i
      motors->m1->setSpeed(0.0);
      motors->m2->setSpeed(0.0);
      motors->m3->setSpeed(0.0);
      motors->m4->setSpeed(0.0);

      bluetooth->writeLine("Complete Test");
   }

   bluetooth->writeLine("Done");

   // tests complete
   for (;;);
}

void Test::AltitudeTest()
{
   // wait for start signal
   waitForStartCommand();

   struct TestParameters rollValues (1.4, 0.4, 0.3,      /*not needed*/ 0); 
   struct TestParameters pitchValues(1.4, 0.4, 0.3,     /*not needed*/ 0);  
   struct TestParameters yawValues  (1.4, 0.4, 0.3,      /*not needed*/ 0);

   struct TestParameters altParams[] = 
   {
      // NOTE: test various kP values with these kI and kD - they seem pretty good
      //TestParameters(kP,   kI,   kD,   gyroWeight)
      //TestParameters(  1.00, 0.10, 0.10, 0.98)
	  //TestParameters(  0.04, 0, 0, 0.98),
	  TestParameters(  0.03, 0.01, 0.005, 0.98)
   };
   float altSetpoint = 16.0;

   const unsigned long iterations = 300; // 700 is about 10 seconds
   int numOfTests = sizeof(altParams)/sizeof(struct TestParameters);

   // char buffer stuff
   char buffer[100];
   char num[10];
   // send the number of tests
   dtostrf(numOfTests, 8, 5, num);
   sprintf(buffer, "Tests: %s", num);
   bluetooth->writeLine(buffer);

   for(int i=0; i<numOfTests; ++i)
   { 
      struct RPYData orientation;
      unsigned long previousTime;
      unsigned long currentTime;
      float deltaTime;
      String message;
      const int WAIT_TIME = 30;

      // Arduino's representation of sprintf does not allow floating points
      // need to convert to char * and then can pass in as a string
      char akP[10];
      char akI[10];
      char akD[10];
      char altSet[10];
	  char alt[10];
      char cnt[10];

      // wait for start signal
      waitForStartCommand();
      
      startupSequence();
      delay(400);

      // convert floats to strings
      dtostrf(altParams[i].kP, 8, 5, akP);
      dtostrf(altParams[i].kI, 8, 5, akI);
      dtostrf(altParams[i].kD, 8, 5, akD);
      sprintf(buffer, "PID values: %s, %s, %s", akP, akI, akD);

      // start keeping track of the timing
      previousTime = millis();

      // last task is to setup the orientation manager with the orientation of the craft before takeoff
      orientationManager->initialRead(orientation);

      // initialize the control system with the initial data
      controllerManager->initialize(orientation, 10.0);

      // setup the roll controller for the test - overwrite whatever ControllerManager::initialize() setup
      controllerManager->getRollController()->setConstants(rollValues.kP, rollValues.kI, rollValues.kD);
      controllerManager->getRollController()->setSetpoint(0.0);
      // setup the pitch controller for the test - overwrite whatever ControllerManager::initialize() setup
      controllerManager->getPitchController()->setConstants(pitchValues.kP, pitchValues.kI, pitchValues.kD);
      controllerManager->getPitchController()->setSetpoint(0.0);
      // setup the yaw controller for the test - overwrite whatever ControllerManager::initialize() setup
      controllerManager->getYawController()->setConstants(yawValues.kP, yawValues.kI, yawValues.kD);
      controllerManager->getYawController()->setSetpoint(180.0);
      // setup the altitude controller for the test - overwrite whatever ControllerManager::initialize() setup
      controllerManager->getAltitudeController()->setConstants(altParams[i].kP, altParams[i].kI, altParams[i].kD);
      controllerManager->getAltitudeController()->setSetpoint(altSetpoint);

      orientationManager->setGyroWeight(altParams[i].gyroWeight);

      // send setup data
      bluetooth->writeLine("Altitude Test"); // name of test
      bluetooth->writeLine("Count: %f Altitude: %f Setpoint: %f"); // expected format of the data - for Matlab
      bluetooth->writeLine(buffer); // test parameters

      // test loop
      for(unsigned long count = 0; count < iterations; )
      {
         currentTime = millis();
         if (currentTime - previousTime >= WAIT_TIME)
         {
            ++count;
			
            // get the delta time in seconds for calculations
            deltaTime = (currentTime - previousTime) / 1000.0;

			   previousTime = currentTime;

            // get current orientation
            RETURN_CODE retVal = orientationManager->getCurrentOrientation(deltaTime, orientation);
			   if (retVal == SUCCESS)
			   {
               Serial1.println(orientation.altitude);
				   // use the current orientation to adjust the motors
				   controllerManager->flightControl(orientation, deltaTime);
				   // check for commands via bluetooth
				   message = bluetooth->readLine();

				   // remove any whitespace
				   message.trim();

				   // handle the message if present
				   if (!message.equals("")) handleMessage(message);

				   // send the current data
				   //dtostrf(count, 8, 0, cnt);
				   //dtostrf(orientation.altitude, 8, 2, alt);
				   //dtostrf(altSetpoint, 8, 2, altSet);
				   //sprintf(buffer, "Count: %s Altitude: %s Setpoint: %s", cnt, alt, altSet);
				   //bluetooth->writeLine(buffer);
            
				   //Serial.println(millis() - currentTime);
			   }
         }
         // wait for 15ms to complete
      }
      // completion of test i
      shutdownSequence();

      bluetooth->writeLine("Complete Test");
   }

   bluetooth->writeLine("Done");

   // tests complete
   for (;;);
}

void Test::handleMessage(String message)
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

void Test::waitForStartCommand()
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

void Test::shutdownSequence()
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

void Test::startupSequence()
{
	 motors->m1->setSpeed(10);
    motors->m2->setSpeed(10);
    motors->m3->setSpeed(10);
    motors->m4->setSpeed(10);
}

void Test::debug()
{
   // wait for start signal
   waitForStartCommand();

   String message;

    // turn on motors
    motors->m1->setSpeed(0.0);
    motors->m2->setSpeed(0.0);
    motors->m3->setSpeed(0.0);
    motors->m4->setSpeed(50.0);
	unsigned long iterations = 0;

    // test loop
    while(iterations < 1000);
    {
        // check for commands via bluetooth
        message = bluetooth->readLine();

        // handle the message if present
        if (!message.equals("")) handleMessage(message); 
		delay(5);
		++iterations;
    }
    // completion of test i
    motors->m1->setSpeed(0.0);
    motors->m2->setSpeed(0.0);
    motors->m3->setSpeed(0.0);
    motors->m4->setSpeed(0.0);

   // tests complete
   for (;;);
}

void Test::debug2()
{
   // wait for start signal
   waitForStartCommand();

   String message;

    // turn on motors
    motors->m1->setSpeed(0.0);
    motors->m2->setSpeed(0.0);
    motors->m3->setSpeed(0.0);
    motors->m4->setSpeed(0.0);

    int iterations = 700;

    for(int i=0; i< 1; ++i)
   { 
      struct RPYData orientation;
      unsigned long previousTime;
      unsigned long currentTime;
      float deltaTime;
      String message;
      const int RATE_200HZ = 15;

      // Arduino's representation of sprintf does not allow floating points
      // need to convert to char * and then can pass in as a string
      char akP[10];
      char akI[10];
      char akD[10];
      char altSet[10];
	  char alt[10];
      char cnt[10];

      // wait for start signal
      waitForStartCommand();

      //// convert floats to strings
      //dtostrf(yawParameters[i].kP, 8, 5, akP);
      //dtostrf(yawParameters[i].kI, 8, 5, akI);
      //dtostrf(yawParameters[i].kD, 8, 5, akD);
      //sprintf(buffer, "PID values: %s, %s, %s", akP, akI, akD);

      // start keeping track of the timing
      previousTime = millis();

      // last task is to setup the orientation manager with the orientation of the craft before takeoff
      orientationManager->initialRead(orientation);

      // initialize the control system with the initial data
      controllerManager->initialize(orientation, 25.0);

      //// setup the roll controller for the test - overwrite whatever ControllerManager::initialize() setup
      //controllerManager->getRollController()->setConstants(rollValues.kP, rollValues.kI, rollValues.kD);
      //controllerManager->getRollController()->setSetpoint(0.0);
      //// setup the pitch controller for the test - overwrite whatever ControllerManager::initialize() setup
      //controllerManager->getPitchController()->setConstants(pitchValues.kP, pitchValues.kI, pitchValues.kD);
      //controllerManager->getPitchController()->setSetpoint(0.0);
      //// setup the yaw controller for the test - overwrite whatever ControllerManager::initialize() setup
      //controllerManager->getYawController()->setConstants(yawParameters[i].kP, yawParameters[i].kI, yawParameters[i].kD);
      //controllerManager->getYawController()->setSetpoint(0.0);
      //// setup the altitude controller for the test - overwrite whatever ControllerManager::initialize() setup
      //controllerManager->getAltitudeController()->setConstants(altitudeValues.kP, altitudeValues.kI, altitudeValues.kD);
      //controllerManager->getAltitudeController()->setSetpoint(altSetpoint);

      //orientationManager->setGyroWeight(yawParameters[i].gyroWeight);

      // send setup data
      //bluetooth->writeLine("Altitude Test"); // name of test
      //bluetooth->writeLine("Count: %f Altitude: %f Setpoint: %f"); // expected format of the data - for Matlab
      //bluetooth->writeLine(buffer); // test parameters

      startupSequence();
      delay(2000);

      // test loop
      for(unsigned long count = 0; count < iterations; )
      {
         currentTime = millis();
         if (currentTime - previousTime >= RATE_200HZ)
         {
            ++count;
			
            // get the delta time in seconds for calculations
            deltaTime = (currentTime - previousTime) / 1000.0;

			   previousTime = currentTime;

            // get current orientation
            RETURN_CODE retVal = orientationManager->getCurrentOrientation(deltaTime, orientation);
			   
            if (retVal == SUCCESS)
			   {
               
               Serial.println(orientation.yaw);

               //Serial1.println(orientation.altitude);
				   
               // use the current orientation to adjust the motors
				   controllerManager->flightControl(orientation, deltaTime);
				   // check for commands via bluetooth
				   message = bluetooth->readLine();

				   // remove any whitespace
				   message.trim();

				   // handle the message if present
				   if (!message.equals("")) handleMessage(message);

				   // send the current data
				   //dtostrf(count, 8, 0, cnt);
				   //dtostrf(orientation.altitude, 8, 2, alt);
				   //dtostrf(altSetpoint, 8, 2, altSet);
				   //sprintf(buffer, "Count: %s Altitude: %s Setpoint: %s", cnt, alt, altSet);
				   //bluetooth->writeLine(buffer);
            
				   //Serial.println(millis() - currentTime);
			   }
         }
         // wait for 15ms to complete
      }
      // completion of test i
      bluetooth->writeLine("Complete Test");
   }
    for (;;);
}
