/*
 * ADC.c
 *
 * Created: 23-Jul-12 4:45:57 PM
 *  Author: M.Hefny
 */ 

#include <avr/io.h>  
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include <avr/interrupt.h> 

#include "typedefs.h"
#include "io_cfg.h"
#include "Commons.h"
#include "ADC.h"

void Init_ADC(void)
{
	DIDR0 	= 0b00111111;	// Digital Input Disable Register - ADC5..0 Digital Input Disable
	ADCSRB 	= 0b00000000; 	// ADC Control and Status Register B - ADTS2:0
}

void read_adc(uint8_t channel)
{
	ADMUX 	= channel;						// Set channel
	ADCSRA 	= 0b11000110;					// ADEN, ADSC, ADPS1,2
	while (ADCSRA & (1 << ADSC));			// Wait to complete
}

