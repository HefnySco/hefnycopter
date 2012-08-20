/*
 * Acc_Sensor.cpp
 *
 * Created: 8/14/2012 2:26:30 AM
 *  Author: hefny
 */ 


#include <avr/io.h> 


#include "../Include/typedefs.h"
#include "../Include/GlobalValues.h"
#include "../Include/Acc_Sensor.h"
#include "../Include/Misc.h"
#include "../Include/IO_config.h"



#define AccLowLimit   450
#define AccHighLimit  850



void Acc_Init(void)
{
	ACC_X  = INPUT;
	ACC_Y  = INPUT;
	ACC_Z  = INPUT;
}





char *Acc_Test(uint8_t channel)
{
	static uint16_t reading;
	
	reading = ADCPort_Get(channel);
	  
	utoa (reading,Result,10);
	
	if ((reading  > AccLowLimit)  
	 || (reading  < AccHighLimit))
	{
		strcpy (&Result[3],&" OK");  
	}
	else
	{
		strcpy (&Result[3],&" FAIL");  
	}
	  
	return Result;
}