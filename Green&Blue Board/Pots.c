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



// value is 10-bits
void ReadGainValues(void)
{
	read_adc(YAW_POT);		   // Read yaw gain ADC5 
	GainInADC[YAW] = ADCW;     // original values from 50 to 1010
	GainInADC[YAW] = GainInADC[YAW] ;

	
	read_adc(ROLL_POT);		   // Read yaw gain ADC5 
	GainInADC[ROLL] = ADCW;
	GainInADC[ROLL] = GainInADC[ROLL] ; 
	
	read_adc(PITCH_POT);		// Read yaw gain ADC5 
	GainInADC[PITCH] = ADCW;
	GainInADC[PITCH] = GainInADC[PITCH] ;
}