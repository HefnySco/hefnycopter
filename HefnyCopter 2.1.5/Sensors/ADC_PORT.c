/*
 * ADC.cpp
 *
 * Created: 8/14/2012 2:30:13 AM
 *  Author: hefny
 */ 


#include <avr/io.h> 
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/pgmspace.h>


#include "../Include/typedefs.h"
#include "../Include/GlobalValues.h"
#include "../Include/ADC_PORT.h"
#include "../Include/IO_config.h"


#if !defined (KK21)

volatile uint16_t ADCValues[8];


/**
 * Called when the voltage conversion is finished
 * 
 *  8.913kHz on mega128 16MHz 1kHz/channel ??
 ***** I used 20 MHz 13 - 260 us conv average = (260-13)/2 + 13 = 136.5 us = 7.326 KHz
*/
ISR(ADC_vect, ISR_NOBLOCK)
{
	static uint8_t _index;
	ADCValues[_index] = ADC;
	_index = (_index + 1) % 8U;
	ADMUX = _index;
	ADCSRA |= _BV(ADSC);
	
	
}


#endif

void ADCPort_Init ()
{
#if defined (KK21)
	DIDR0 	= 0b11111111;					// Digital Input Disable Register - ADC0~7 Digital Input Disable
	ADCSRB 	= 0b00000000; 					// ADC Control and Status Register B - ADTS2:0
#else
	DIDR0 = 0xFF;	// disable all digital inputs on Port A
	ADMUX = 0;
	ADCSRA = 0b11011111;	// ADEN, ADSC, ADIE, 
#endif
	

		
}


/*
uint16_t ADCPort_Read(uint8_t channel)
{
	// disable interrupt and wait for end of conversion
	ADCSRA = (0xFF & ~_BV(ADIE));
	loop_until_bit_is_clear(ADCSRA, ADSC);
	
	ADMUX = channel;		// set channel to read from
	ADCSRA |= _BV(ADSC);	// start conversion
	loop_until_bit_is_clear(ADCSRA, ADSC);	// wait to complete
	ADCSRA = 0b11011111;
	return ADC;
}
*/
inline uint16_t ADCPort_Get(uint8_t channel)
{
#if defined (KK21)
	ADMUX = channel;					// Set channel - use Aref as reference

	ADCSRA 	= 0b11000110;					// ADEN, ADSC, ADPS1,2
	while (ADCSRA & (1 << ADSC));			// Wait to complete. Result is in ADCW
	return ADCW;
#else
	
	uint16_t _t;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		_t = ADCValues[channel];
	}
	return _t;
#endif
}

