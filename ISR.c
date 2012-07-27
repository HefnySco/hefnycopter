/*
 * ISR.c
 *
 * Created: 22-Jul-12 9:52:26 PM
 *  Author: M.Hefny
 */ 


/* ----------- Main Code -----------  */

#include <avr/io.h>  
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <avr/interrupt.h> 


#include "typedefs.h"
#include "io_cfg.h"
#include "Commons.h"
#include "motors.h"
#include "ISR.h"



ISR (TIMER0_COMPA_vect)
{
	OCR0A_X -=1;
	if ((OCR0A_X==0))
	{
		
				
	}	
}

ISR(TIMER1_OVF_vect)
{
	
	TCNT1_X +=1; // each step is 1u x 0xffff = 0.065536
}



ISR(TIMER2_OVF_vect)
{
	
	TCNT2_X +=1; // each step is 1u x 0xffff = 0.065536
	/*if (TCNT2_X==10)
	{
		
		LED= ~LED;
		TCNT2_X=0;
	}*/
}

#ifndef CPPM_MODE  	// Normal RX mode

ISR(PCINT2_vect)
{
	if (RX_ROLL)				// RX_ROLL rising
	{
			RxChannel1Start = TCNT1;
	} 
	else
	{	
			if (TCNT1 < RxChannel1Start) 
			{
				RxChannel1 = 0xffff - RxChannel1Start + TCNT1;
			}				
			else
			{
				RxChannel1 = TCNT1 - RxChannel1Start;
			}				
	}
	RxChannelsUpdatedFlag = true;
}

ISR(INT0_vect)
{
	if (RX_PITCH)	// Rising 
	{
		RxChannel2Start = TCNT1;
	} 
	else 
	{				// Falling
		RxChannel2 = TCNT1 - RxChannel2Start;
	}
	RxChannelsUpdatedFlag = true;
}

ISR(INT1_vect)
{
	if (RX_COLL)	// Rising
	{
		RxChannel3Start = TCNT1;
	} 
	else 
	{				// Falling
		RxChannel3 = TCNT1 - RxChannel3Start;
	}
	RxChannelsUpdatedFlag = true;
}

ISR(PCINT0_vect)
{
	if (RX_YAW)	// Rising
	{
		RxChannel4Start = TCNT1;
	} 
	else 
	{				// Falling
		RxChannel4 = TCNT1 - RxChannel4Start;
	}
	RxChannelsUpdatedFlag = true;
}

//************************************************************
// PPM RX mode 	- input on PD2 (INT0/elevator)
// NB: JR/Spectrum channel order (Th,Ai,El,Ru,5,6,7,8)
// 	   Other brands of TX will lead to the wrong channels
//	   being decoded into the NeXtcopter RxChannel variables
//	   unless they are changed here.
//************************************************************

#else
ISR(INT0_vect)
{	// Check to see if previous period was a sync pulse
	// If so, reset channel number
	if ((TCNT1 - PPMSyncStart) > SYNCPULSEWIDTH) ch_num = 0;
	PPMSyncStart = TCNT1;
	switch(ch_num)
	{
		case 0:
			RxChannel1Start = TCNT1;
			ch_num++;
			break;
		case 1:
			RxChannel2Start = TCNT1;
			RxChannel3 = TCNT1 - RxChannel1Start;	// Ch3 - Throttle
			RxChannelsUpdatedFlag = true;			// Flag that data changed
			ch_num++;
			break;
		case 2:
			RxChannel3Start = TCNT1;
			RxChannel1 = TCNT1 - RxChannel2Start;	// Ch1 - Aileron
			RxChannelsUpdatedFlag = true;			// Flag that data changed
			ch_num++;
			break;
		case 3:
			RxChannel4Start = TCNT1;
			RxChannel2 = TCNT1 - RxChannel3Start;	// Ch2 - Elevator
			RxChannelsUpdatedFlag = true;			// Flag that data changed
			ch_num++;
			break;
		case 4:
			RxChannel5Start = TCNT1;
			RxChannel4 = TCNT1 - RxChannel4Start;	// Ch4 - Rudder
			RxChannelsUpdatedFlag = true;			// Flag that data changed
			ch_num++;
			break;
		case 5:
			RxChannel6Start = TCNT1;
			RxChannel5 = TCNT1 - RxChannel5Start;	// Ch5 - Gear
			ch_num++;
			break;
		case 6:
			RxChannel7Start = TCNT1;
			RxChannel6 = TCNT1 - RxChannel6Start;	// Ch6 - Flap 
			ch_num++;
			break;
		case 7:
			RxChannel8Start = TCNT1;
			RxChannel7 = TCNT1 - RxChannel7Start;	// Ch7 - AUX2
			ch_num++;
			break;
		case 8:
			RxChannel8 = TCNT1 - RxChannel8Start;	// Ch8 - AUX3
			ch_num++;
			break;
		default:
			break;
	} // Switch
} // ISR(INT0_vect)

#ifdef PROX_MODULE
ISR(INT1_vect)
{
	if (ECHO)		// Echo pulse just started
	{
		EchoTimeStart = TCNT1;
	} 
	else 
	{				// Echo pulse completed
		EchoTime = TCNT1 - EchoTimeStart;
	}
} // ISR(INT1_vect)
#endif
#endif
