/************************************************/
/* Written by:  Aaron Derstine                  */
/* Date:        1/11/14                         */
/* Name:        main.cpp                        */
/* Description: This is the main routine which  */
/*    basically is the automatically generated  */
/*    Arduino routine but with the quadrotor    */
/*    class                                     */
/************************************************/
#define TESTING;
#include <Arduino.h>
#include "Quadrotor.h"

int main(void)
{
   // Arduino init function
	init();

#if defined(USBCON)
	USBDevice.attach();
#endif
	
   // initialization sequence
	Quadrotor quadrotor;
   quadrotor.initialize();
    
   // infinite loop where control functionality
	for (;;) {
		quadrotor.loop();
		if (serialEventRun) serialEventRun();
	}
        
	return 0;
}

