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

volatile int16_t  iTemp16; // used by Receiver intrrupts

/*
FailSafe:

	if you open the TX & then close it the RX Throttle signal will be valid even after shutting down TX.
	if you open the RX and TX is not open all signals will be valid except Throttle.
*/



// RX_GOOD How does it work?
// I checked this behavior on Orange Receivers CH6.
// When TX remote is turned OFF the only valid signal is THR others are no signals.
// so I monitor a dead signal on any of RX except THR this could be any one... I chose YAW.... 
// I detect the lost signal in the RX_GetReceiverThrottleValue() because it is called less 1/4 times than the other function RX_GetReceiverValue()
volatile uint16_t RX_raw[RXChannels];
volatile uint16_t RX_LastValidSignal_timestamp;
volatile uint16_t RX_LastValidSignal_timestampAux;

#define RX_Div_Factor	16	// div by 16



void CalculateSignalLength(uint8_t ChannelIndex)
{
	if (TCNT1 > RX_raw[ChannelIndex] )
	{
		RX_raw[ChannelIndex] = TCNT1 - RX_raw[ChannelIndex] ;	
	}
	else
	{
		RX[ChannelIndex] = (0xffff - RX_raw[ChannelIndex] + TCNT1 );	
	}
		
}

#ifdef SECONDARY_INPUT_RX
 
 uint8_t OldPortCValue;

__attribute__ ((section(".lowtext")))
ISR (RX_ALL_vect)
{
	uint8_t Changes = PINC ^ OldPortCValue;
	OldPortCValue = PINC;
	LED_Orange = ~LED_Orange;
	
	if ((Changes & RX_ROLL_PIN)!=0)
	{
		if (RX_ROLL)
		{
			RX_raw[RXChannel_AIL]=TCNT1;
		}
		else
		{
			CalculateSignalLength (RXChannel_AIL);
			RX_LastValidSignal_timestampAux = TCNT1_X;
			RX_Good = TX_FOUND_ERR;
		}	
	}
	if ((Changes & RX_PITCH_PIN)!=0)
	{
		if (RX_PITCH)
		{
			RX_raw[RXChannel_ELE]=TCNT1;
		}
		else
		{
			CalculateSignalLength (RXChannel_ELE);
		}	
	}
	if ((Changes & RX_COLL_PIN)!=0)
	{
		if (RX_COLL)
		{
			RX_raw[RXChannel_THR]=TCNT1;
		}
		else
		{
			CalculateSignalLength (RXChannel_THR);
			RX_LastValidSignal_timestamp = TCNT1_X;
			RX_Good = TX_CONNECTED_ERR;
		}	
	}
	if ((Changes & RX_YAW_PIN) !=0)
	{
		if (RX_YAW)
		{
			RX_raw[RXChannel_RUD]=TCNT1;
		}
		else
		{
			CalculateSignalLength (RXChannel_RUD);
			
		}	
	}
	
}
#endif

#ifdef PRIMARY_INPUT_RX

__attribute__ ((section(".lowtext")))
ISR (RX_COLL_vect)
{
	if (RX_COLL)
	{
		RX_raw[RXChannel_THR]=TCNT1;
	}
	else
	{
		CalculateSignalLength(RXChannel_THR);
		
		RX_LastValidSignal_timestamp = TCNT1_X;
		RX_Good = TX_CONNECTED_ERR;
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
		CalculateSignalLength(RXChannel_AIL);
		
		RX_LastValidSignal_timestampAux = TCNT1_X;
		RX_Good = TX_FOUND_ERR;

		
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
		CalculateSignalLength(RXChannel_ELE);
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
		CalculateSignalLength(RXChannel_RUD);
	}
}
#endif

__attribute__ ((section(".lowtext")))
ISR (RX_AUX_vect)
{
	if (RX_AUX)
	{
		RX_raw[RXChannel_AUX]=TCNT1;
	}
	else
	{
		CalculateSignalLength(RXChannel_AUX);
	}
}


void RX_Init(void)
{
	
	RX_ROLL_DIR 		= INPUT;
	RX_PITCH_DIR 		= INPUT;
	RX_COLL_DIR   		= INPUT;
	RX_YAW_DIR   	 	= INPUT;
	RX_AUX_DIR   	 	= INPUT;
	
	RX_Good =TX_NOT_FOUND;
	
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		RX_LastValidSignal_timestamp= TCNT1_X;
		RX_LastValidSignal_timestampAux= TCNT1_X;
	}		
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
	//////if (RX_Good != TX_GOOD) return 0;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
		_t = ((int)(RX[Channel] - Config.RX_Mid[Channel])) / RX_Div_Factor;

	return _t;
}

/*
// Retrieves Throttle value [0 - 1000].
// IMPORTANT: if throttle signal is accidentally -or using trim- less than Config.RX_Min[RXChannel_THR] then the returned value will be roll back
// to 0xffff range which means an aggressive Quad copter action. we avoid this by using a signed int.
*/
int16_t RX_GetReceiverThrottleValue ()
{
	
	///if (RX_Good != TX_GOOD) return 0;
	
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		if ( (TCNT1_X - RX_LastValidSignal_timestamp) > 20)
		{
			RX_Good =TX_NOT_FOUND;
			return 0;
		}	
		
		if ( (TCNT1_X - RX_LastValidSignal_timestampAux) > 20)
		{
			RX_Good =TX_DISCONNECTED;
			return 0;
		}	
		
		
		iTemp16 = ((int)(RX[RXChannel_THR] - Config.RX_Min[RXChannel_THR])) / RX_Div_Factor;
	}		
	
	return iTemp16;
}
 
void RX_CopyLatestReceiverValues (void)
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
			if ((RX_Latest[i]<STICK_DEADBAND) && (RX_Latest[i]>-STICK_DEADBAND)) RX_Latest[i]=0;
		}
		
	}		
	
}

void RX_StickCenterCalibration (void)
{
	
	uint16_t tempRX;
	for (int i=0;i<RXChannels;++i)
	{
		tempRX = RX_raw_GetReceiverValues(i);
		if (tempRX!=0)
		{
			
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
}