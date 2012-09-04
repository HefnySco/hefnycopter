/*
 * Acc_Sensor.cpp
 *
 * Created: 8/14/2012 2:26:30 AM
 *  Author: hefny
 */ 


#include <avr/io.h> 
#include <avr/pgmspace.h>
#include <avr/delay.h>

#include <string.h>
#include <stdlib.h>

#include "../Include/typedefs.h"
#include "../Include/GlobalValues.h"
#include "../Include/Sensors.h"
#include "../Include/Misc.h"
#include "../Include/IO_config.h"



//limits for sensor testing
#define AccLowLimit			450
#define AccHighLimit		850
#define GyroLowLimit		500		
#define GyroHighLimit		630


P_STR strOK[] ="OK";
P_STR strFail[] ="Fail";
	


void Sensors_Init(void)
{
	ACC_X  = INPUT;
	ACC_Y  = INPUT;
	ACC_Z  = INPUT;
	
	GYRO_X = INPUT;
	GYRO_Y = INPUT;
	GYRO_Z = INPUT;
}




char *Sensors_Gyro_Test(uint8_t channel)
{
	static uint16_t reading;
	
	reading = ADCPort_Get(channel);
	  
	utoa (reading,Result,10);
	
	if ((reading  > GyroLowLimit)  
	 || (reading  < GyroHighLimit))
	{
		strcpy (&Result[3],&strOK);  
	}
	else
	{
		strcpy (&Result[3],&strFail);  
	}
	  
	return Result;
}


/*
// This function Test if sensors are working OK or not.
*/
char *Sensors_Acc_Test(uint8_t channel)
{
	
	
	nResult[channel] = ADCPort_Get(channel);
	  
	utoa (nResult[channel],Result,10);
	
	if ((nResult[channel]  > AccLowLimit)  
	 || (nResult[channel]  < AccHighLimit))
	{
		strcpy (&Result[3],strOK);  
	}
	else
	{
		strcpy (&Result[3],&strFail);  
	}
	  
	return Result;
}


void CalibrateGyro (void)
{
	
}

void CalibrateAcc (void)
{
	
}

/*
// Calibrate Sensors and return result in nResult global variable.
*/
void Sensors_Calibrate (void)
{
//	memset(&nResult,1,14);
	BOOL LEDOLD = LED_Orange;
	int i;
	for (i=0;i<6;++i)
	{
		nResult [i]=0;
	}
	
	for (int i=0;i<25;++i)
	{
		nResult[ACC_X_Index] += ADCPort_Get(ACC_X_PNUM);
		nResult[ACC_Y_Index] += ADCPort_Get(ACC_Y_PNUM);
		nResult[ACC_Z_Index] += ADCPort_Get(ACC_Z_PNUM);
		
		nResult[GYRO_X_Index] += ADCPort_Get(GYRO_X_PNUM);
		nResult[GYRO_Y_Index] += ADCPort_Get(GYRO_Y_PNUM);
		nResult[GYRO_Z_Index] += ADCPort_Get(GYRO_Z_PNUM);
		
		_delay_ms(10);
		LED_Orange =~LED_Orange;
	}
	
	LED_Orange = LEDOLD;
	
	for (i=0;i<6;++i)
	{
		nResult[i] /=25;
	}	
}

void Sensors_ReadAll (void)
{
	Sensors_Latest[ACC_X_Index] = ADCPort_Get(ACC_X_PNUM);
	Sensors_Latest[ACC_Y_Index] = ADCPort_Get(ACC_Y_PNUM);
	Sensors_Latest[ACC_Z_Index] = ADCPort_Get(ACC_Z_PNUM);
		
	Sensors_Latest[GYRO_X_Index] = ADCPort_Get(GYRO_X_PNUM);
	Sensors_Latest[GYRO_Y_Index] = ADCPort_Get(GYRO_Y_PNUM);
	Sensors_Latest[GYRO_Z_Index] = ADCPort_Get(GYRO_Z_PNUM);
	
	Sensors_Latest[V_BAT_Index] = Sensor_GetBattery();
}


char * Sensor_GetBatteryTest(void)
{
		// Write Voltage
	nTemp16 = Sensor_GetBattery();
	utoa(nTemp16 /10,Result,10);
	strcat_P(Result,PSTR("."));
	utoa(nTemp16 %10,Result2,10);
	strcat (Result,Result2);
	return Result;
}

inline uint16_t  Sensor_GetBattery(void)
{
	return ADCPort_Get(V_BAT_PNUM) * 100 / 372;
} 