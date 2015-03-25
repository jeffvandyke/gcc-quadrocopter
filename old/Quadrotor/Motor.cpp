// Motor.cpp

#include "Motor.h"

// pwm signal for motors has resolution of 255
// 255 is max
// 127 is 50% duty cycle representing off
#define SPEED_SCALER 1.28
#define SPEED_OFFSET 125

Motor::Motor()
{
   spin  = CLOCKWISE;
   speed = 0.0;
   id    = UNSET;
   pin   = 13;
}

void Motor::initialize(Spin spin, MotorID id)
{
   this->spin = spin;
   this->id   = id;

   switch (id)
   {		
      case ONE:
         pin = 2;
         break;
      case TWO:
         pin = 3;
         break;
      case THREE:
         pin = 5;
         break;
      case FOUR:
         pin = 6;
         break;
      default:
         pin = 13;
         break;
   };

   setSpeed(0.0);

}

float Motor::getSpeed() const
{
   return speed;
}

RETURN_CODE Motor::setSpeed(float value)
{
   // ensure motor is initialized
   if (id == UNSET) return UNINITIALIZED;

   // ensure it is within acceptable range
   if (value >= 0.0 && value <= 100.0)
   {
      speed = value;

      // convert a percentage value into a 255 resolution for the pwm signal offset by 127
      int pwm = floor(SPEED_SCALER * value) + SPEED_OFFSET;
      analogWrite(pin, pwm);
      return SUCCESS;
   }

   if (value > 100.0) return setSpeed(100.0);
   if (value < 0.0)   return setSpeed(0.0);
   return INVALID;
}

MotorID Motor::getId() const
{
   return id;
}

Spin Motor::getSpin() const
{
   return spin;
}