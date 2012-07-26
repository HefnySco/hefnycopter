/*
 * Gyro.c
 *
 * Created: 24-Jul-12 9:12:56 PM
 *  Author: M.Hefny
 */ 

/* ----------- Main Code -----------  */

#include <avr/io.h>  
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include <avr/interrupt.h> 

#include "typedefs.h"
#include "io_cfg.h"
#include "Commons.h"
#include "Gyro.h"


/*
//	Output of this function is new values for gyroZero[].
// gyroZero[] represents ZERO values of gyros when quad is stable.
*/
void CalibrateGyros(void)
{
	uint8_t i;

	gyroZero[ROLL] 	= 0;						
	gyroZero[PITCH] = 0;	
	gyroZero[YAW] 	= 0;

	for (i=0;i<32;i++)					// Calculate average over 32 reads
	{
		read_adc(ROLL_GYRO);			// Read roll gyro ADC2
		gyroADC[ROLL] = ADCW;
		read_adc(PITCH_GYRO);			// Read pitch gyro ADC1
		gyroADC[PITCH] = ADCW;
		read_adc(YAW_GYRO);				// Read yaw gyro ADC0
		gyroADC[YAW] = ADCW;

		gyroZero[ROLL] 	+= gyroADC[ROLL];						
		gyroZero[PITCH] += gyroADC[PITCH];	
		gyroZero[YAW] 	+= gyroADC[YAW];

		_delay_ms(10);					// Get a better gyro average over time
	}

	gyroZero[ROLL] 	= (gyroZero[ROLL] >> 5);	//Divide by 32				
	gyroZero[PITCH] = (gyroZero[PITCH] >> 5);
	gyroZero[YAW] 	= (gyroZero[YAW]>> 5);

	GyroCalibrated = true;
}


/*
// Output of this function is gyroADC[] set to new values.
// values are normalized by subtracting gyroZero[]
*/
void ReadGyros(void)
{
	int16_t gyro;

	read_adc(ROLL_GYRO);				// Read roll gyro ADC2
	gyro = ADCW;
	gyro -= gyroZero[ROLL]; 			// Remove offset from gyro output
#ifndef MEMS_MODULE
	gyroADC[ROLL] = -gyro;				// Reverse gyro on KK boards
#else
	gyroADC[ROLL] = gyro;				// Normal gyro on MEMS module
#endif

	read_adc(PITCH_GYRO);				// Read pitch gyro ADC1
	gyro = ADCW;
	gyro -= gyroZero[PITCH]; 			// Remove offset from gyro output
#ifndef MEMS_MODULE
	gyroADC[PITCH] = -gyro;				// Reverse gyro on KK boards
#else
	gyroADC[PITCH] = gyro;				// Normal gyro on MEMS module
#endif

	read_adc(YAW_GYRO);					// Read yaw gyro ADC0
	gyro = ADCW;
	gyro -= gyroZero[YAW]; 				// Remove offset from gyro output
	gyroADC[YAW] = gyro;				// Normal gyro on all boards
	
}
