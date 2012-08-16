/*
 * Beeper.cpp
 *
 * Created: 8/13/2012 11:06:47 PM
 *  Author: hefny
 */ 


#include <avr/io.h>

#include "../Include/Beeper.h"
#include "../Include/Misc.h"
#include "../Include/IO_config.h"

Beeper::Beeper()
{
	
}

void Beeper::Beep (uint16_t msDuration, uint8_t Times)
{
	
	bool CurrentBuzzer_Status = Buzzer;
	
	for (int i=0; i< Times; ++i)
	{
		Buzzer = ~Buzzer;
		delay_ms(msDuration);
		Buzzer = ~Buzzer;
		delay_ms(msDuration);		
	}
	
	Buzzer = CurrentBuzzer_Status;
	
}