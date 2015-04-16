// Bluetooth.cpp

#include "Bluetooth.h"

Bluetooth::~Bluetooth()
{
   Serial1.end();
}

void Bluetooth::initialize()
{
   // Serial1 uses pins 19(RX and should be connected to bluetooth TX) and 18(TX which should be connected to bluetooth RX)
   Serial1.begin(115200, SERIAL_8N1); // baud rate = 115200, 8 data bits, no parity, 1 stop bit
   Serial1.println("R,1"); // restart it
}

String Bluetooth::readLine()
{
   if(Serial1.available() > 0) // available, returns the number of bytes available to read or -1 if none are available
   {
      return Serial1.readString();
   }
   else return ""; // return an empty string
}

void Bluetooth::println(String dataOut)
{
   Serial1.println(dataOut);
}

void Bluetooth::print(int dataOut)
{
   Serial1.print(dataOut);
}

void Bluetooth::calibrateLoop()
{
   for (;;)
   {
      if(Serial1.available())  // If the bluetooth sent any characters
      {
         // Send any characters the bluetooth prints to the serial monitor
         Serial.print((char)Serial1.read());  
      }
      if(Serial.available())  // If stuff was typed in the serial monitor
      {
         // Send any characters the Serial monitor prints to the bluetooth
         Serial1.print((char)Serial.read());
      }
      // and loop forever and ever!
   }
}