/*
 * Acc_Sensor.cpp
 *
 * Created: 8/14/2012 2:26:30 AM
 *  Author: hefny
 */ 


#include <avr/io.h> 
#include <util/atomic.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include <string.h>
#include <stdlib.h>

#include "../Include/typedefs.h"
#include "../Include/GlobalValues.h"
#include "../Include/Sensors.h"
#include "../Include/Misc.h"
#include "../Include/IO_config.h"

#if defined(KK21)
#include "../Include/I2C.h"
#include "../include/MPU6050.h"
#endif


#if defined(KK21)


void init_i2c_gyros(void)
{
	// First, configure the MPU6050
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS , MPU60X0_RA_PWR_MGMT_1, 0x80); 
	delay_ms(5);
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS , MPU60X0_RA_PWR_MGMT_1, 0x01); // Gyro X clock, awake
  	// Other regs cannot be written until the MPU6050 is out of sleep mode
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS , MPU60X0_RA_CONFIG, Config.MPU6050_LPF); 	// 0x06 = 5Hz, (5)10Hz, (4)20Hz, (3)42Hz, (2)98Hz, (1)188Hz LPF
	
	// Now configure gyros
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS , MPU60X0_RA_GYRO_CONFIG, Config.MPU6050_Gyro_Range); // 500 deg/sec
}

void init_i2c_accs(void)
{
	// Wake MPU6050
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS , MPU60X0_RA_PWR_MGMT_1, 0x01); // Gyro X clock, awake
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS , MPU60X0_RA_ACCEL_CONFIG, Config.MPU6050_Acc_Range); //default is  4G full scale
}
#endif


void Sensors_Init(void)
{
	#if defined (KK21)
	init_i2c_gyros();
	init_i2c_accs();
	
	return ;
	#else
	
	ACC_PITCH  = INPUT;
	ACC_ROLL  = INPUT;
	ACC_Z  = INPUT;
	
	GYRO_ROLL = INPUT;
	GYRO_PITCH = INPUT;
	GYRO_Z = INPUT;
	#endif
}



/*
// This function Test if sensors are working OK or not.
*/
char *Sensors_Test(uint8_t channel, uint16_t LowLimit ,uint16_t HighLimit)
{
#if defined (KK21)
	
	Sensors_ReadAll();
	nResult[channel] = Sensors_Latest[channel];
	
#else
	nResult[channel] = ADCPort_Get(channel);
	
#endif 
	 
	utoa (nResult[channel],Result,10);
	
	if ((nResult[channel]  >= LowLimit)  
	 && (nResult[channel]  <= HighLimit))
	{
		strcat (Result, ("  "));  
	}
	else
	{
		strcat (Result, (" X"));  
	}
	
	return Result;
	
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
	for (i=0;i<25;++i)
	{

#if defined (KK21)
			get_raw_gyros();   // read sensors without subtracting the offset.
			get_raw_accs();   // read sensors without subtracting the offset.
	
		   for (int s=0;s<SENSORS_ALL;++s)
		  {
			nResult[s] += Sensors_Latest[s];
		  }			  
#else
			for (int s=0;s<SENSORS_ALL;++s)
		  {
			nResult[s] += ADCPort_Get(SensorsIndex[s]);		
		  }
#endif

		_delay_ms(40);
		LED_Orange =~LED_Orange;
	}
	
	LED_Orange = LEDOLD;
	
	for (i=0;i<6;++i)
	{
		Config.Sensor_zero[i]  = (double)nResult[i] /25.0;
	}	
	
	Config.IsCalibrated = (Config.IsCalibrated | CALIBRATED_SENSOR);
	
	//Config.Sensor_zero[ACC_Z_Index]	-= 120; // ACC_Z reads 120 by default.
	
	//nResult[ACC_Z_Index]-=100; // Sensor: horizontal, upward ... the caller of this function is responsible for updating Config.Sensor_zero[i] = nResult[i];
	
}





void get_raw_gyros(void)
{
// Get data from MPU6050 for KK2.1
#ifdef KK21
	uint8_t Gyros[6];
	int16_t RawADC[3];

	// For KK2.1 boards, use the i2c data from the MPU6050
	// Check gyro array axis order and change in io_cfg.h
	readI2CbyteArray (MPU60X0_DEFAULT_ADDRESS,MPU60X0_RA_GYRO_XOUT_H,(uint8_t *)Gyros,6);

	// Reassemble data into gyroADC array and down-sample to reduce resolution and noise
	RawADC[PITCH] = (int16_t)(((int16_t)Gyros[0]) << 8 | Gyros[1]) >> 4 ;					// Gyro X
	RawADC[ROLL]  = (int16_t)(((int16_t)Gyros[2]) << 8 | Gyros[3]) >> 4 ;					// Gyro Y
	RawADC[YAW]	  = (int16_t)(((int16_t)Gyros[4]) << 8 | Gyros[5]) >> 4 ;					// Gyro Z
	

	// Reorient the data as per the board orientation
	Sensors_Latest[GYRO_ROLL_Index] 	= RawADC[ROLL] ; //RawADC[(int8_t)pgm_read_byte(&Gyro_RPY_Order[Config.Orientation][ROLL])];
	Sensors_Latest[GYRO_PITCH_Index] 	= RawADC[PITCH] ; //RawADC[(int8_t)pgm_read_byte(&Gyro_RPY_Order[Config.Orientation][PITCH])];
	Sensors_Latest[GYRO_Z_Index]		= RawADC[YAW] ; //RawADC[(int8_t)pgm_read_byte(&Gyro_RPY_Order[Config.Orientation][YAW])];

#else
	for (int i=0;i<3;++i)  // gyro
	{
		Sensors_Latest[i] = ADCPort_Get(SensorsIndex[i])-Config.Sensor_zero[i]; 
		if (abs(Sensors_Latest[i]) <= DEAD_BAND_GYRO) Sensors_Latest[i]=0;
	}
#endif
}



void get_raw_accs(void)
{
// Get data from MPU6050 for KK2.1
#ifdef KK21
	uint8_t Accs[6];
	int16_t RawADC[3];

	// For KK2.1 boards, use the i2c data from the MPU6050
	readI2CbyteArray(MPU60X0_DEFAULT_ADDRESS,MPU60X0_RA_ACCEL_XOUT_H,(uint8_t *)Accs,6);

	// Reassemble data into accADC array and down sample to reduce resolution and noise
	// This notation is true to the chip, but not the board orientation
	RawADC[PITCH] = (int16_t)(((int16_t)Accs[0]) << 8 | Accs[1]) >> 3 ;					// Accel X
	RawADC[ROLL]  = (int16_t)(((int16_t)Accs[2]) << 8 | Accs[3]) >> 3 ;					// Accel Y
	RawADC[YAW]	  = (int16_t)(((int16_t)Accs[4]) << 8 | Accs[5]) >> 3 ;					// Accel Z
	
	//temp1 = (int16_t) Accs[2] << 8;					// Accel Y
	//temp2 = Accs[3];
	//RawADC[ROLL] = (int16_t)(temp1 + temp2) >> 6;

	
	// Reorient the data as per the board orientation
	Sensors_Latest[ACC_ROLL_Index] 		= RawADC[ROLL] ; //RawADC[(int8_t)pgm_read_byte(&ACC_RPY_Order[Config.Orientation][ROLL])];
	Sensors_Latest[ACC_PITCH_Index] 	= RawADC[PITCH]; //RawADC[(int8_t)pgm_read_byte(&ACC_RPY_Order[Config.Orientation][PITCH])];
	Sensors_Latest[ACC_Z_Index]			= RawADC[YAW] ; //RawADC[(int8_t)pgm_read_byte(&ACC_RPY_Order[Config.Orientation][YAW])];

#else
	//	For the KK2.0, the order of the analog sensors is swapped in adc.c
	for (int i=3;i<6;++i)  //ACC
	{
		Sensors_Latest[i] = ADCPort_Get(SensorsIndex[i])-Config.Sensor_zero[i]; 
		//if (abs(Sensors_Latest[i]) <= DEAD_BAND_GYRO) Sensors_Latest[i]=0;
	}

#endif
}


#define DEAD_BAND_GYRO	3
//uint32_t LastLoopTime[2];
//uint16_t TX,TX1,TX2;
void Sensors_ReadAll (void)
{
 #if defined(KK21)
 
  get_raw_accs();
  get_raw_gyros();
  
  for (int i=0;i<6;++i)  // gyro
	{
		Sensors_Latest[i] = Sensors_Latest[i] - Config.Sensor_zero[i]; 
		if ((i < 3) && (abs(Sensors_Latest[i]) <= DEAD_BAND_GYRO)) Sensors_Latest[i]=0;
	}
    
 #endif
  
  Sensors_Latest[V_BAT_Index] = Sensor_GetBattery(); 
	
}


int16_t  Sensor_GetBattery(void)
{
	
	
	#if defined (KK21)
	
	return (ADCPort_Get(V_BAT_PNUM) *  BAT_VOLT_RATIO);
	
	//MHefny:2.1.5
	#else
	 // because the V_BAT is connected to a voltage divider R1 & R2
	return (ADCPort_Get(V_BAT_PNUM) *  BAT_VOLT_RATIO);
	
	#endif
	
} 


//inline void DynamicCalibration (void)
//{
	///* 
	//// Dynamic calibration of ACC
	//*/
	//if ((Sensors_Latest[ACC_PITCH_Index] >= ACC_MIN) && (Sensors_Latest[ACC_PITCH_Index] < ACC_MAX))
	//{
		//StabilityMatrix_GX[Sensors_Latest[ACC_PITCH_Index]-ACC_MIN]+=1;
	//}
	//if ((Sensors_Latest[ACC_ROLL_Index] >= ACC_MIN) && (Sensors_Latest[ACC_ROLL_Index] < ACC_MAX))
	//{
		//StabilityMatrix_GY[Sensors_Latest[ACC_ROLL_Index]-ACC_MIN]+=1;
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
	//ACC_Pitch_Offset = maxX + ACC_MIN;	/* Range from -10 to 9 */
	//ACC_Roll_Offset = maxY + ACC_MIN;	/* Range from -10 to 9 */
//}