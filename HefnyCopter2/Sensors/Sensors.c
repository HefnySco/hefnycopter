/*
 * Acc_Sensor.cpp
 *
 * Created: 8/14/2012 2:26:30 AM
 *  Author: hefny
 */ 


#include <avr/io.h> 
#include <util/atomic.h>
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
	
	//for (int i=0; i<20;++i)
	//{
		//
		//StabilityMatrix_GX[i]=0;
		//StabilityMatrix_GY[i]=0;
	//}
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
  //return arctan2(-Sensors_Latest[ACC_Z_Index],-Sensors_Latest[Acc_Index]) + 256;    // in Quid: 1024/(2*PI))
//arctan2(accelData[PITCH] * sign[PITCH], sqrt((long(accelData[ROLL]) * accelData[ROLL]) + (long(accelData[ZAXIS]) * accelData[ZAXIS])));
	//return arctan2 (-Sensors_Latest[Acc_Index] , sqrt((long())))
	//if (Acc_Index == ACC_X_Index) return arctan2(-Sensors_Latest[ACC_X_Index],Sensors_Latest[ACC_Z_Index]+100) * 0.4275;
	//if (Acc_Index == ACC_Y_Index) return arctan2(-Sensors_Latest[ACC_Y_Index],Sensors_Latest[ACC_Z_Index]+100) * 0.4275;
	if (Acc_Index == ACC_X_Index) return arctan2(Sensors_Latest[ACC_X_Index],(int16_t)CompGyroZ+100) * 0.84609375;
	if (Acc_Index == ACC_Y_Index) return arctan2(Sensors_Latest[ACC_Y_Index],(int16_t)CompGyroZ+100) * 0.84609375;
	
}

// divide by 10 after reading
int16_t Sensors_GetGyroRate(int8_t Gyro_Index) {
	int16_t t= Sensors_Latest[Gyro_Index];	                                            // ARef=3.3V, Gyro sensitivity=2mV/(deg/sec)
	if ((t<=1) && (t>=-1)) return 0;
  return (int16_t)(t * 0.4583333333);							// in quid/sec:(1024/360)/1024 * 3.3/0.002)
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
		
		_delay_ms(40);
		LED_Orange =~LED_Orange;
	}
	
	LED_Orange = LEDOLD;
	
	for (i=0;i<6;++i)
	{
		nResult[i] /=25;
	}	
	
	Config.IsCalibrated = (Config.IsCalibrated | CALIBRATED_SENSOR);
	for (i=0;i<6;++i)
	Config.Sensor_zero[i] = nResult[i];
		
	
	//nResult[ACC_Z_Index]-=100; // Sensor: horizontal, upward ... the caller of this function is responsible for updating Config.Sensor_zero[i] = nResult[i];
		
}

#define DEAD_BAND_GYRO	2
uint32_t LastLoopTime[2];
uint16_t TX,TX1,TX2;
void Sensors_ReadAll (void)
{
   
   uint16_t *T;
   ATOMIC_BLOCK(ATOMIC_FORCEON)
   {
		TX= TCNT1;
		TX1= TCNT1_X;
   }   

	//T = &LastLoopTime[0];
	//T[0]= TX;
	//T[1]= TX1;
	
	Sensors_Latest[ACC_X_Index] = ADCPort_Get(ACC_X_PNUM)-Config.Sensor_zero[ACC_X_Index]; // - ACC_X_Offset == Gyro Y in negative Direction
	Sensors_Latest[ACC_Y_Index] = ADCPort_Get(ACC_Y_PNUM)-Config.Sensor_zero[ACC_Y_Index]; // - ACC_Y_Offset == Gyro X in negative Direction
	Sensors_Latest[ACC_Z_Index] = ADCPort_Get(ACC_Z_PNUM)-Config.Sensor_zero[ACC_Z_Index];
		
	Sensors_Latest[GYRO_X_Index] = ADCPort_Get(GYRO_X_PNUM)-Config.Sensor_zero[GYRO_X_Index];
	if (abs(Sensors_Latest[GYRO_X_Index])< DEAD_BAND_GYRO) Sensors_Latest[GYRO_X_Index] = 0;
	Sensors_Latest[GYRO_Y_Index] = ADCPort_Get(GYRO_Y_PNUM)-Config.Sensor_zero[GYRO_Y_Index];
	if (abs(Sensors_Latest[GYRO_Y_Index])< DEAD_BAND_GYRO) Sensors_Latest[GYRO_Y_Index] = 0;
	Sensors_Latest[GYRO_Z_Index] = ADCPort_Get(GYRO_Z_PNUM)-Config.Sensor_zero[GYRO_Z_Index];
	if (abs(Sensors_Latest[GYRO_Z_Index])< DEAD_BAND_GYRO) Sensors_Latest[GYRO_Z_Index] = 0;
	
	Sensors_Latest[V_BAT_Index] = Sensor_GetBattery(); 
	
	
	
	// Handle the odd case where the TCNT1 rolls over and LastLoopTime[0] < LastLoopTime[1]
	/*
	if (LastLoopTime[0] > LastLoopTime[1])
		Sensors_dt = LastLoopTime[0] - LastLoopTime[1];
	else 
		Sensors_dt = (0xffffffff - LastLoopTime[1]) + LastLoopTime[0] ;
		
	LastLoopTime[1] = LastLoopTime[0]; // in 100 us unit
	*/
	if (TX2 > TX1)
	{
		Sensors_dt = (0xffff - TX1) +  TX2;
	}
	else
	{
		Sensors_dt = TX1 - TX2;
	}
	TX2=TX1;
}


/*char * Sensor_GetBatteryTest(void)
{
		// Write Voltage
	nTemp16 = Sensor_GetBattery();
	utoa(nTemp16 /10,Result,10);
	strcat_P(Result,PSTR("."));
	utoa(nTemp16 %10,Result2,10);
	strcat (Result,Result2);
	return Result;
}*/

int16_t  Sensor_GetBattery(void)
{
	 // because the V_BAT is connected to a voltage divider R1 & R2
	return (ADCPort_Get(V_BAT_PNUM) *  BAT_VOLT_RATIO);
} 


//inline void DynamicCalibration (void)
//{
	///* 
	//// Dynamic calibration of ACC
	//*/
	//if ((Sensors_Latest[ACC_X_Index] >= ACC_MIN) && (Sensors_Latest[ACC_X_Index] < ACC_MAX))
	//{
		//StabilityMatrix_GX[Sensors_Latest[ACC_X_Index]-ACC_MIN]+=1;
	//}
	//if ((Sensors_Latest[ACC_Y_Index] >= ACC_MIN) && (Sensors_Latest[ACC_Y_Index] < ACC_MAX))
	//{
		//StabilityMatrix_GY[Sensors_Latest[ACC_Y_Index]-ACC_MIN]+=1;
	//}
	//
	//uint16_t maxX=0, maxY=0;
		//
	//for (int i=0; i<20;++i)
	//{
		//if (StabilityMatrix_GX[i]> StabilityMatrix_GX[maxX])
		//{
			//maxX = i;
		//}
		//
		//if (StabilityMatrix_GY[i]> StabilityMatrix_GY[maxY])
		//{
			//maxY = i;
		//}
		//
	//}
	//
	//ACC_X_Offset = maxX + ACC_MIN;	/* Range from -10 to 9 */
	//ACC_Y_Offset = maxY + ACC_MIN;	/* Range from -10 to 9 */
//}