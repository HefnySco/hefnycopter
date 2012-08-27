/*
 * Misc.cpp
 *
 * Created: 8/14/2012 1:04:48 AM
 *  Author: hefny
 */ 

#include <util/delay.h>
#include <util/atomic.h>

#include "../Include/Misc.h"


//////////////////////////////// TIMING
//
//void delay_us(uint16_t time)
//{
	//while(time--) 
	//{ 
		//
		//for (int i=0;i<200;++i)
		//{
			//asm ("nop");
		//}
		////_delay_us(1); 
	//}
//}
//
//void delay_ms(uint16_t time)
//{
	//while(time--) 
	//{ 
		//_delay_us(1000); 
	//}
//}


void delay_ms(uint16_t times)            /* time delay for us */
{ 
	times *=10;
	int8_t TimeOut;
	int8_t Multi;
	while(times > 0)
   {
	   for (Multi=0;Multi<10;++Multi)
	   {
		    if (TCNT1H > (255-20))
			{
				TimeOut = 0xff-TCNT1H;
			}
			else
			{
				TimeOut = TCNT1H + 20;	
			}
			while (TCNT1H < TimeOut); 
			times-=1;
	   }		
   }
}

void delay_us(uint16_t time)
{
	
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


