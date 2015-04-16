#include "PIDcontrol.h"

//starts the setpoint at zero
	PIDController()
	{
		setPoint.xPosition=0; 
		setPoint.yPosition=0;
		setPoint.zPosition=0;
		setPoint.xVelocity=0;
		setPoint.yVelocity=0;
		setPoint.zVelocity=0;
		setPoint.xAcceleration=0; 
		setPoint.yAcceleration=0;
		setPoint.zAcceleration=0;

		setPoint.xAngle=0; 
		setPoint.yAngle=0;
		setPoint.zAngle=0;
		setPoint.xRotation=0; 
		setPoint.yRotation=0;
		setPoint.zRotation=0;
	}

   ~PIDController()
   {
   }

   void PIDController::move(quadState_t deltaPos)
   {
	   	setPoint.xPosition+=deltaPos.xPosition; 
		setPoint.yPosition+=deltaPos.yPosition;
		setPoint.zPosition+=deltaPos.zPosition;

		setPoint.zRotation+=deltaPos.zRotation;
   }

   quadState_t PIDController::PIDUpdate(quadState_t newVal)
   {
   }

	int calculateOutput(int value)
	{
		// basis for this code was found at http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
		//Same as last year's code
		// current error
		int error = setpoint - Value;

		// proportional term
		int pTerm = kP * error;

		// integral term
		integralTermSum += error * deltaTime;

		// derivative term
		int dTerm = kD * (currentValue - previousInput) / deltaTime;

		// update internal data
		previousInput = currentValue;

		int pTerm + kI * integralTermSum - dTerm;
	}
	quadState_t setPoint;
	
	int deltaTime;