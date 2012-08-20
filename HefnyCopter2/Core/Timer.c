/*
 * Timer.c
 *
 * Created: 8/18/2012 6:56:30 AM
 *  Author: hefny
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>

#include "../Include/IO_config.h"
#include "../Include/GlobalValues.h"
#include "../Include/Timer.h"


volatile uint32_t x=0;

ISR(TIMER1_OVF_vect)
{
	x = x+1;
	TCNT1_X++;
	
}


void Timer_Init(void)
{
	// Timer1 @20MHz with overflow interrupt
	TCCR1B = _BV(CS10);
	TIMSK1 = _BV(TOIE1);
	TCNT1_X =0;
}