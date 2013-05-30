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
	CLK		20000000			
						
Timer		Scale	Frequency Hz	Tick time (sec)	OV Time		TCNT_X OV	1 Sec Count
Timer 0		8		2500000			0.00000040		0.0001024		
Timer 1		1		20000000		0.00000005		0.0000128	0.0032768	305
Timer1H		256		78125			0.00001280		0.0032768		
Timer 2		32		625000			0.00000160		0.0004096	0.1048576	10

*/

//volatile uint32_t x=0;

ISR(TIMER1_OVF_vect)
{
	
	TCNT1_X++; // click every 0.0032768 sec [1 sec = 305.17578125 TCNT1_X] overflows every 214.7483648 sec = 3.579 min
	//TCNT1_X_GlobalTimer +=32;  // 0.0032768
}


ISR(TIMER2_OVF_vect)
{
	TCNT2_X++; 
}	


void Timer_Init(void)
{
	
	//TIMER ZERO is RESERVED for LCD DISPLAY.	
	
	// Timer1 @20MHz with overflow interrupt
	TCCR1B = _BV(CS10);
	TIMSK1 = _BV(TOIE1);
	TCNT1_X =0;




	// timer2 8bit - run @ 20MHz / 32 = 312.5 KHz = 3.2us
	// and Stick-Arming
	//TCCR2A  = (1 << WGM22);	
	/*
	For Timer 2 
			CS22 CS21 CS20 Description
			0 0 0		No clock source (Timer/Counter stopped).
			0 0 1		clkT2S/(No prescaling)
			0 1 0		clkT2S/8 (From prescaler)
			0 1 1		clkT2S/32 (From prescaler)
			1 0 0		clkT2S/64 (From prescaler)
			1 0 1		clkT2S/128 (From prescaler)
			1 1 0		clkT2S/256 (From prescaler)
			1 1 1		clkT2S/1024 (From prescaler)
	*/
	TCCR2B  = (1 << CS21) | (1 << CS20);	//  div by 32	
	TIMSK2  = _BV(TOIE2); 
	TIFR2   = 0;
	TCNT2	= 0;		// this overflows every  1.6us x 0xff = 409.6 us,  value tick 1.6us
	TCNT2_X = 0;
	/* IMPORTANT TCNT2 is used completly by motor and is reset during ESC pulse generation so you cannot depend on it for other purpose
	unless it is not time critical and no accuracy is required such as UI delays*/
	// tick: 12.8u  ov:3.2768ms	: TCNT2_X OV:  214.7483648 sec

}