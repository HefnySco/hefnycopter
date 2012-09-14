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



/*
// This function Test if sensors are working OK or not.
*/
char *Sensors_Test(uint8_t channel, uint16_t LowLimit ,uint16_t HighLimit)
{
	
	nResult[channel] = ADCPort_Get(channel);
	  
	utoa (nResult[channel],Result,10);
	
	if ((nResult[channel]  > LowLimit)  
	 || (nResult[channel]  < HighLimit))
	{
		strcpy (&Result[3],strOK);  
	}
	else
	{
		strcpy (&Result[3],&strFail);  
	}
	  
	return Result;
}


  
int Sensors_GetAccAngle(int8_t Acc_Index) {
  return arctan2(-Sensors_Latest[ACC_Z_Index], -Sensors_Latest[Acc_Index]) + 256;    // in Quid: 1024/(2*PI))
}

int Sensors_GetGyroRate(int8_t Gyro_Index) {                                         // ARef=3.3V, Gyro sensitivity=2mV/(deg/sec)
  return (int)(Sensors_Latest[Gyro_Index] * 4.583333333);							// in quid/sec:(1024/360)/1024 * 3.3/0.002)
}

/*
// Calibrate Sensors and return result in nResult global variable.
*/
void Sensors_Calibrate (void)
{
	BOOL LEDOLD = LED_Orange;
	int i;
	for (i=0;i<6;++i)
	{
		nResult [i]=0;
	}
	
	// check: http://www.x-firm.com/?page_id=191
	
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
	
	nResult[ACC_Z_Index]-=100; // Sensor: horizontal, upward
}

void Sensors_ReadAll (void)
{
	Sensors_Latest[ACC_X_Index] = ADCPort_Get(ACC_X_PNUM)-Config.Sensor_zero[ACC_X_Index];
	Sensors_Latest[ACC_Y_Index] = ADCPort_Get(ACC_Y_PNUM)-Config.Sensor_zero[ACC_Y_Index];
	Sensors_Latest[ACC_Z_Index] = ADCPort_Get(ACC_Z_PNUM)-Config.Sensor_zero[ACC_Z_Index];
		
	Sensors_Latest[GYRO_X_Index] = ADCPort_Get(GYRO_X_PNUM)-Config.Sensor_zero[GYRO_X_Index];
	Sensors_Latest[GYRO_Y_Index] = ADCPort_Get(GYRO_Y_PNUM)-Config.Sensor_zero[GYRO_Y_Index];
	Sensors_Latest[GYRO_Z_Index] = ADCPort_Get(GYRO_Z_PNUM)-Config.Sensor_zero[GYRO_Z_Index];
	
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