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

/*
Timer		Scale	Tick time	OV Time		TCNT_X OV		"1 Sec Count"
Timer 0		8		0.0000004								2,500,000
Timer 1		1		0.000000050	0.0032768	214.7483648		20,000,000
Timer1H		256		0.000012800	0.0032768					78,125
Timer 2		32		0.0000016	0.0004096	26.8435456		625,000
*/

//volatile uint32_t x=0;

ISR(TIMER1_OVF_vect)
{
	
	TCNT1_X++; // click every 0.0032768 sec [1 sec = 305.17578125 TCNT1_X] overflows every 214.7483648 sec = 3.579 min
	
}


ISR(TIMER2_OVF_vect)
{
	TCNT2_X++; //TCNT2  overflows every  3.2us x 0xff = 0.0008192 sec,  TCNT2_X value tick every 819.2 us and overflows every 53 sec.
}	


void Timer_Init(void)
{
	
	//TIMER ZERO is RESERVED for LCD DISPLAY.	
	
	// Timer1 @20MHz with overflow interrupt
	TCCR1B = _BV(CS10);
	TIMSK1 = _BV(TOIE1);
	TCNT1_X =0;




	// timer2 8bit - run @ 20MHz / 64 = 312.5 KHz = 3.2us
	// and Stick-Arming
	//TCCR2A  = (1 << WGM22);	
	TCCR2B  = (1 << CS21) | (1 << CS20);	//  div by 32 == 3.2 us tick
	TIMSK2  = _BV(TOIE2); 
	TIFR2   = 0;
	TCNT2	= 0;		// this overflows every  1.6us x 0xff = 409.6 us,  value tick 1.6us
	TCNT2_X = 0;

	// tick: 12.8u  ov:3.2768ms	: TCNT2_X OV:  214.7483648 sec

}