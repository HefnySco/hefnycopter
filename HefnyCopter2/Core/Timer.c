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
	TCNT1_X++; // click every 0.0032768 sec [1 sec = 305.17578125 TCNT1_X]
	
}


ISR(TIMER2_OVF_vect)
{

	TCNT2_X++; //TCNT2  overflows every  3.2us x 0xff = 0.0008192 sec,  TCNT2_X value tick every 8192 us

}	


void Timer_Init(void)
{
	// Timer1 @20MHz with overflow interrupt
	TCCR1B = _BV(CS10);
	TIMSK1 = _BV(TOIE1);
	TCNT1_X =0;




	// timer2 8bit - run @ 20MHz / 64 = 312.5 KHz = 3.2us
	// and Stick-Arming
	TCCR2A  = (1 << WGM22);	
	TCCR2B  = (1 << CS21) | (1 << CS20);	//  div by 64 == 3.2 us tick
	TIMSK2  = _BV(TOIE2); 
	TIFR2   = 0;
	TCNT2	= 0;		// this overflows every  3.2us x 0xff = 0.0008192 sec,  value tick 3.2us 
	TCNT2_X=0;


}