/*
 * Misc.cpp
 *
 * Created: 8/14/2012 1:04:48 AM
 *  Author: hefny
 */ 


#include "../Include/Misc.h"


//////////////////////////////// TIMING

void delay_us(uint8_t time)            /* time delay for us */
{ 
	 while(time--)
	 {
		asm volatile ("NOP"); asm volatile ("NOP"); 
		asm volatile ("NOP"); asm volatile ("NOP"); 
		asm volatile ("NOP"); asm volatile ("NOP"); 
		asm volatile ("NOP"); 
	 }
}

void delay_ms(uint16_t time)
{
	uint8_t i;
	while(time--)
	{
		for(i=0;i<10;i++) delay_us(100);
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


