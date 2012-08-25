/*
 * Receiver.c
 *
 * Created: 8/18/2012 6:40:59 AM
 *  Author: hefny
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "Include/typedefs.h"
#include "Include/GlobalValues.h"
#include "Include/IO_config.h"
#include "Include/Receiver.h"

volatile uint16_t RX_raw[RXChannels];

#define RX_Div_Factor	16	// div by 16

__attribute__ ((section(".lowtext")))
ISR (RX_COLL_vect)
{
	if (RX_COLL)
	{
		RX_raw[RXChannel_THR]=TCNT1;
	}
	else
	{
		if (TCNT1 > RX_raw[RXChannel_THR] )
		{
			RX_raw[RXChannel_THR] = TCNT1 - RX_raw[RXChannel_THR] ;	
		}
		else
		{
			RX[RXChannel_THR] = (0xffff - RX_raw[RXChannel_THR] + TCNT1 );	
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
			RX[RXChannel_AIL] = (0xffff - RX_raw[RXChannel_AIL] + TCNT1 );	
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
			RX[RXChannel_ELE] = (0xffff - RX_raw[RXChannel_ELE] + TCNT1);	
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
			RX[RXChannel_RUD] = (0xffff - RX_raw[RXChannel_RUD] + TCNT1);	
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
			RX[RXChannel_AUX] = (0xffff - RX_raw[RXChannel_AUX] + TCNT1);	
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

void RX_StickCenterCalibrationInit(void)
{
	for (int i=0; i<RXChannels; ++i)
	{
		RX_MAX_raw[i]=0;
		RX_MIN_raw[i]=0xfffe;
	}
}


  uint16_t RX_raw_GetReceiverValues (uint8_t Channel)
{
	uint16_t _t;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
		_t = RX[Channel];
	return _t;
}

 int16_t RX_GetReceiverValues (uint8_t Channel)
{
	int16_t _t;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
		_t = ((int)(RX[Channel] - Config.RX_Mid[Channel])) / RX_Div_Factor;

	return _t;
}

/*
// Retrives Throttle value [0 - 1000].
// IMPORTANT: if throttle signal is accidently -or using trim- less than Config.RX_Min[RXChannel_THR] then the returned value will be roll back
// to 0xffff range which means an agressive Quadcopter action. we avoid this by using a signed int.
*/
int16_t RX_GetReceiverThrottleValue ()
{
	int16_t _t;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
		_t = ((int)(RX[RXChannel_THR] - Config.RX_Min[RXChannel_THR])) / RX_Div_Factor;
	
	return _t;
}

void RX_CopyReceiverValues (void)
{
	for (int i=0;i<RXChannels;++i)
	{
		if (i == RXChannel_THR)
		{
			RX_Latest[i]= RX_GetReceiverThrottleValue(i);	 
		}
		else
		{
			RX_Latest[i]= RX_GetReceiverValues(i);	 
		}
		
	}		
	
}

void RX_StickCenterCalibration (void)
{
	
	uint16_t tempRX;
	for (int i=0;i<RXChannels;++i)
	{
		tempRX = RX_raw_GetReceiverValues(i);
		if ( tempRX > RX_MAX_raw[i]) 
		{
			RX_MAX_raw[i] = tempRX;
		}
		else if (tempRX < RX_MIN_raw[i]) 
		{
			RX_MIN_raw[i] = tempRX;
		}
	}
}