/*
 * LED.cpp
 *
 * Created: 8/14/2012 1:02:09 AM
 *  Author: hefny
 */ 

#include <avr/io.h> 

#include "../Include/typedefs.h"
#include "../Include/LED.h"
#include "../Include/Misc.h"
#include "../Include/IO_config.h"


void LED_FlashOrangeLED (uint16_t msDuration, uint8_t Times)
{
	bool CurrentLED_Status = LED_Orange;
	
	for (int i=0; i< Times; ++i)
	{
		LED_Orange = ~LED_Orange;
		delay_ms(msDuration);
		LED_Orange = ~LED_Orange;
		delay_ms(msDuration);		
	}
	
	LED_Orange = CurrentLED_Status;
	
}