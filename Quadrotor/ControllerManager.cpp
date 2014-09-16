// ControllerManager.cpp

#include "ControllerManager.h"

#define MAX_MOTOR_SPEED 51.0
#define MAX_ALTITUDE_ADJUSTMENT 0.5
#define MOTOR_DIFFERENCE 4.0

ControllerManager::ControllerManager()
{
   rollController     = new PIDController;
   pitchController    = new PIDController;
   yawController      = new PIDController;
   altitudeController = new PIDController;

   currentMotorMax = 0.0;
}

ControllerManager::~ControllerManager()
{
   if (rollController)     delete rollController;
   if (pitchController)    delete pitchController;
   if (yawController)      delete yawController;
   if (altitudeController) delete altitudeController;
}

void ControllerManager::initialize(const struct RPYData &initialData, float startingSpeed)
{
   // give the initial starting data to the pid controllers
   rollController    ->initialize(initialData.roll);
   pitchController   ->initialize(initialData.pitch);
   yawController     ->initialize(initialData.yaw);
   altitudeController->initialize(initialData.altitude);

   // set PID values for the roll controller
   rollController->setConstants(1.6, 0.2, 0.6);
   rollController->setSetpoint(0.0);

   // set PID values for the pitch controller
   pitchController->setConstants(1.6, 0.2, 0.6);
   pitchController->setSetpoint(0.0);

   // set PID values for the yaw controller
   yawController->setConstants(1.4, 0.4, 0.3);
   yawController->setSetpoint(180.0);

   // set PID values for the altitude controller
   altitudeController->setConstants(0.05, 0.01, 0.01);
   altitudeController->setSetpoint(12.0);

   currentMotorMax = startingSpeed;
}

void ControllerManager::attachMotors(Motor *motor1, Motor *motor2, Motor *motor3, Motor *motor4)
{
   this->motor1 = motor1;
   this->motor2 = motor2;
   this->motor3 = motor3;
   this->motor4 = motor4;
}

void ControllerManager::flightControl(const struct RPYData &currentOrientation, float deltaTime)
{
   // calculate what needs to happen to correct for disturbance on each axis
   float rollCorrection     = rollController    ->calculateOutput(currentOrientation.roll, deltaTime);
   float pitchCorrection    = pitchController   ->calculateOutput(currentOrientation.pitch, deltaTime);
   float yawCorrection      = yawController     ->calculateOutput(currentOrientation.yaw, deltaTime);
   float altitudeCorrection = altitudeController->calculateOutput(currentOrientation.altitude, deltaTime);

   adjustMotorValues(rollCorrection, pitchCorrection, yawCorrection, altitudeCorrection);
}

void ControllerManager::flightControl(const struct RPYData &currentOrientation, float deltaTime, struct ControllerOutput &output)
{
   float rollCorrection     = rollController    ->calculateOutput(currentOrientation.roll, deltaTime);
   float pitchCorrection    = pitchController   ->calculateOutput(currentOrientation.pitch, deltaTime);
   float yawCorrection      = yawController     ->calculateOutput(currentOrientation.yaw, deltaTime);
   float altitudeCorrection = altitudeController->calculateOutput(currentOrientation.altitude, deltaTime);

   adjustMotorValues(rollCorrection, pitchCorrection, yawCorrection, altitudeCorrection);

   output.roll     = rollCorrection;
   output.pitch    = pitchCorrection;
   output.yaw      = yawCorrection;
   output.altitude = altitudeCorrection;
}

void ControllerManager::adjustMotorValues(float rollCorrection, float pitchCorrection, float yawCorrection, float altitudeCorrection)
{
   float m1speed = motor1->getSpeed();
   float m2speed = motor2->getSpeed();
   float m3speed = motor3->getSpeed();
   float m4speed = motor4->getSpeed(); 

   // adjust each motor based on what correction is needed
   m1speed += pitchCorrection;
   m3speed -= pitchCorrection;

   m2speed -= rollCorrection;
   m4speed += rollCorrection;

   // adjust for yaw
   m1speed -= yawCorrection;
   m3speed -= yawCorrection;

   m2speed += yawCorrection;
   m4speed += yawCorrection;

   // find currentMotorMax value
   if (m1speed > currentMotorMax) currentMotorMax = m1speed;
   if (m2speed > currentMotorMax) currentMotorMax = m2speed;
   if (m3speed > currentMotorMax) currentMotorMax = m3speed;
   if (m4speed > currentMotorMax) currentMotorMax = m4speed;

   // set the motor min value based on the difference
   float motorMin = currentMotorMax - MOTOR_DIFFERENCE;

   // make sure it doesn't go too high
   if(currentMotorMax > MAX_MOTOR_SPEED)
   {
      currentMotorMax = MAX_MOTOR_SPEED;
      motorMin = MAX_MOTOR_SPEED - MOTOR_DIFFERENCE;
   }

   // pitch axis motors
   m1speed = (m1speed > currentMotorMax) ? currentMotorMax : (m1speed < motorMin) ? motorMin : m1speed;
   m3speed = (m3speed > currentMotorMax) ? currentMotorMax : (m3speed < motorMin) ? motorMin : m3speed;

   // roll axis motors
   m2speed = (m2speed > currentMotorMax) ? currentMotorMax : (m2speed < motorMin) ? motorMin : m2speed;
   m4speed = (m4speed > currentMotorMax) ? currentMotorMax : (m4speed < motorMin) ? motorMin : m4speed;

   // make sure the altitude correction is not too great - want gradual adjustments
   // since altitude correction occurs last want it to be able to slow down past what
   // the roll, pitch, yaw correction requires so descending requires a bit more of an adjustment
   if      (altitudeCorrection >    MAX_ALTITUDE_ADJUSTMENT) altitudeCorrection =    MAX_ALTITUDE_ADJUSTMENT;
   else if (altitudeCorrection < -5*MAX_ALTITUDE_ADJUSTMENT) altitudeCorrection = -5*MAX_ALTITUDE_ADJUSTMENT;

   // adjust every motor speed for altitude correction
   m1speed += altitudeCorrection;
   m2speed += altitudeCorrection;
   m3speed += altitudeCorrection;
   m4speed += altitudeCorrection;

   motor1->setSpeed(m1speed);
   motor2->setSpeed(m2speed);
   motor3->setSpeed(m3speed);
   motor4->setSpeed(m4speed);
}


