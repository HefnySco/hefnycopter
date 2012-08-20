/*
 * Receiver.c
 *
 * Created: 8/18/2012 6:40:59 AM
 *  Author: hefny
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#include "Include/typedefs.h"
#include "Include/GlobalValues.h"
#include "Include/IO_config.h"
#include "Include/Receiver.h"

volatile uint16_t RX_raw[RXChannels];

__attribute__ ((section(".lowtext")))
ISR (RX_COLL_vect)
{
	if (RX_COLL)
	{
		RX_raw[RXChannel_THR]=TCNT1;
		LED_Orange= ~LED_Orange;	
	}
	else
	{
		if (TCNT1 > RX_raw[RXChannel_THR] )
		{
			RX_raw[RXChannel_THR] = TCNT1 - RX_raw[RXChannel_THR] ;	
		}
		else
		{
			RX[RXChannel_THR] = 0xffff - RX_raw[RXChannel_THR] + TCNT1;	
		}
		
	}
	
}



__attribute__ ((section(".lowtext")))
ISR (RX_ROLL_vect)
{
	if (RX_ROLL)
	{
		RX_raw[RXChannel_AIL]=TCNT1;
	}
	else
	{
		if (TCNT1 > RX_raw[RXChannel_AIL] )
		{
			RX_raw[RXChannel_AIL] = TCNT1 - RX_raw[RXChannel_AIL] ;	
		}
		else
		{
			RX[RXChannel_AIL] = 0xffff - RX_raw[RXChannel_AIL] + TCNT1;	
		}
		
	}
	
}


__attribute__ ((section(".lowtext")))
ISR (RX_PITCH_vect)
{
	if (RX_PITCH)
	{
		RX_raw[RXChannel_ELE]=TCNT1;
	}
	else
	{
		if (TCNT1 > RX_raw[RXChannel_ELE] )
		{
			RX_raw[RXChannel_ELE] = TCNT1 - RX_raw[RXChannel_ELE] ;	
		}
		else
		{
			RX[RXChannel_ELE] = 0xffff - RX_raw[RXChannel_ELE] + TCNT1;	
		}
		
	}
}


__attribute__ ((section(".lowtext")))
ISR (RX_YAW_vect)
{
	if (RX_YAW)
	{
		RX_raw[RXChannel_RUD]=TCNT1;
	}
	else
	{
		if (TCNT1 > RX_raw[RXChannel_RUD] )
		{
			RX_raw[RXChannel_RUD] = TCNT1 - RX_raw[RXChannel_RUD] ;	
		}
		else
		{
			RX[RXChannel_RUD] = 0xffff - RX_raw[RXChannel_RUD] + TCNT1;	
		}
		
	}
}


__attribute__ ((section(".lowtext")))
ISR (RX_AUX_vect)
{
	if (RX_AUX)
	{
		RX_raw[RXChannel_AUX]=TCNT1;
	}
	else
	{
		if (TCNT1 > RX_raw[RXChannel_AUX] )
		{
			RX_raw[RXChannel_AUX] = TCNT1 - RX_raw[RXChannel_AUX] ;	
		}
		else
		{
			RX[RXChannel_AUX] = 0xffff - RX_raw[RXChannel_AUX] + TCNT1;	
		}
		
	}
}


void RX_Init(void)
{
	RX_ROLL_DIR 		= INPUT;
	RX_PITCH_DIR 		= INPUT;
	RX_COLL_DIR   		= INPUT;
	RX_YAW_DIR   	 	= INPUT;
	RX_AUX_DIR   	 	= INPUT;
}

void RX_GetReceiverValues (void)
{
	
}


void RX_StickCenterCalibration (void)
{
	
	
}