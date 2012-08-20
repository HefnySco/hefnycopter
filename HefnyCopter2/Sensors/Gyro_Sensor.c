/*
 * Gyro_Sensor.cpp
 *
 * Created: 8/14/2012 2:37:11 AM
 *  Author: hefny
 */ 


#include <avr/io.h> 
#include <string.h>
#include <avr/pgmspace.h>


#include "../Include/typedefs.h"
#include "../Include/GlobalValues.h"
#include "../Include/Gyro_Sensor.h"
#include "../Include/Misc.h"
#include "../Include/IO_config.h"

//limits for sensor testing
#define GyroLowLimit 500		
#define GyroHighLimit 630


void Gyro_Init(void)
{
	GYRO_X = INPUT;
	GYRO_Y = INPUT;
	GYRO_Z = INPUT;
}


P_STR strOK[] ="OK";
P_STR strFail[] ="Fail";
	
	

char *Gyro_Test(uint8_t channel)
{
	static uint16_t reading;
	
	reading = ADCPort_Get(channel);
	  
	utoa (reading,Result,10);
	
	if ((reading  > GyroLowLimit)  
	 || (reading  < GyroHighLimit))
	{
		strcpy (&Result[3],&" OK");  
	}
	else
	{
		strcpy (&Result[3],&" FAIL");  
	}
	  
	return Result;
}