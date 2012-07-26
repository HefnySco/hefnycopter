/*
 * Pots.c
 *
 * Created: 24-Jul-12 9:39:36 PM
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
#include "Pots.h"


void GetVbat(void)			// Get battery voltage (VBAT on ADC6)
{	
	read_adc(VBAT);			// Multiplication factor = (Display volts / 1024) / (Vbat / 11 / Vref)

#ifndef MEMS_MODULE
	vBat = ADCW * 7 / 4;	// For Vref = 1.64V, factor = 1.76 (7/4 = 1.75)
#else
	vBat = ADCW * 31 / 12;	// For Vref = 2.40V, factor = 2.578 (31/12 = 2.583)
#endif
}


void ReadGainValues(void)
{
	read_adc(YAW_POT);		// Read yaw gain ADC5 
	GainInADC[YAW] = ADCW;

	read_adc(ROLL_POT);		// Read yaw gain ADC5 
	GainInADC[ROLL] = ADCW;

	read_adc(PITCH_POT);		// Read yaw gain ADC5 
	GainInADC[PITCH] = ADCW;

}