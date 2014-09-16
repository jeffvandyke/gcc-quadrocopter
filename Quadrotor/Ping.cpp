// Ping.cpp

#include "Ping.h"

Ping::Ping(int pin) : pin(pin)
{

}

Ping::~Ping()
{

}

float Ping::getDistanceInches()
{
   // establish variables for duration of the ping
   unsigned long duration;

   // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
   // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
   pinMode(pin, OUTPUT);
   digitalWrite(pin, LOW);
   delayMicroseconds(2);
   digitalWrite(pin, HIGH);
   delayMicroseconds(5);
   digitalWrite(pin, LOW);

   // The same pin is used to read the signal from the PING))): a HIGH
   // pulse whose duration is the time (in microseconds) from the sending
   // of the ping to the reception of its echo off of an object.
   pinMode(pin, INPUT);
   duration = pulseIn(pin, HIGH);

   // convert the time into a distance
   // According to Parallax's datasheet for the PING))), there are
   // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
   // second).  This gives the distance travelled by the ping, outbound
   // and return, so we divide by 2 to get the distance of the obstacle.
   // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
   return duration / 73.746 / 2.0;
}