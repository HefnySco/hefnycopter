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






ISR(ADC_vect, ISR_NOBLOCK)
{
	static uint8_t _index;
	ADCValues[_index] = ADC;
	_index = (_index + 1) % 8U;
	ADMUX = _index;
	ADCSRA |= _BV(ADSC);
}

void ADCPort_Init ()
{
	DIDR0 = 0xFF;	// disable all digital inputs on Port A
	ADMUX = 0;
	ADCSRA = 0b11011111;	// ADEN, ADSC, ADIE, 
		
}



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

inline uint16_t ADCPort_Get(uint8_t channel)
{
	uint16_t _t;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
		_t = ADCValues[channel];
	return _t;
}