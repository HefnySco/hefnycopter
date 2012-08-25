/*
 * Misc.cpp
 *
 * Created: 8/14/2012 1:04:48 AM
 *  Author: hefny
 */ 

#include <util/delay.h>

#include "../Include/Misc.h"


//////////////////////////////// TIMING

void delay_us(uint16_t time)
{
	while(time--) 
	{ 
		_delay_us(1); 
	}
}

void delay_ms(uint16_t time)
{
	while(time--) 
	{ 
		_delay_ms(1); 
	}
}

//////////////////////////////// EOF - TIMIMG


//////////////////////////////// WIRING
/*
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
	uint8_t value = 0;
	uint8_t i;

	for (i = 0; i < 8; ++i) {
		digitalWrite(clockPin, HIGH);
		if (bitOrder == LSBFIRST)
			value |= digitalRead(dataPin) << i;
		else
			value |= digitalRead(dataPin) << (7 - i);
		digitalWrite(clockPin, LOW);
	}
	return value;
}
*/
//////////////////////////////// EOF - Wiring


