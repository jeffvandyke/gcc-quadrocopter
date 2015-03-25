// QuadrotorAPI.cpp

#include "QuadrotorAPI.h"

void sendErrorMessage(RETURN_CODE code, const char *functionName)
{
   char buffer[100];
   sprintf(buffer, "Code: %d in function: %s", code, functionName);
   Serial1.println(buffer);   
}

