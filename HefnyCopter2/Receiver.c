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

#define RX_SIGNAL_LOST_DURATION	20

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
volatile uint16_t RX_raw[2][RXChannels];
volatile uint16_t RX1_LastValidSignal_timestamp;
volatile uint16_t RX1_LastValidSignal_timestampAux;
volatile uint16_t RX2_LastValidSignal_timestamp;
volatile uint16_t RX2_LastValidSignal_timestampAux;

#define RX_Div_Factor	16	// div by 16


void CalculateSignalLength1(uint8_t ChannelIndex)
{
	if (TCNT1 > RX_raw[0][ChannelIndex] )
	{
		RX_Length[0][ChannelIndex] = TCNT1 - RX_raw[0][ChannelIndex] ;	
	}
	else
	{
		RX_Length[0][ChannelIndex] = (0xffff - RX_raw[0][ChannelIndex] + TCNT1 );	
	}
	
}
void CalculateSignalLength2(uint8_t ChannelIndex)
{
	if (TCNT1 > RX_raw[1][ChannelIndex] )
	{
		RX_Length[1][ChannelIndex] = TCNT1 - RX_raw[1][ChannelIndex] ;	
	}
	else
	{
		RX_Length[1][ChannelIndex] = (0xffff - RX_raw[1][ChannelIndex] + TCNT1 );	
	}
	
}

#ifdef SECONDARY_INPUT_RX
 
 uint8_t OldPortCValue;

__attribute__ ((section(".lowtext")))
ISR (RX2_ALL_vect)
{
	uint8_t Changes = PINC ^ OldPortCValue;
	OldPortCValue = PINC;
	
	if ((Changes & RX2_ROLL_PIN)!=0)
	{
		if (RX2_ROLL)
		{
			RX_raw[1][RXChannel_AIL]=TCNT1;
		}
		else
		{
			CalculateSignalLength2 (RXChannel_AIL);
			
			RX2_LastValidSignal_timestampAux = TCNT1_X;
			RX_Good = TX2_FOUND_ERR;  // CLR bit 6 ---  Status = OK
		}	
	}
	if ((Changes & RX2_PITCH_PIN)!=0)
	{
		if (RX2_PITCH)
		{
			RX_raw[1][RXChannel_ELE]=TCNT1;
		}
		else
		{
			CalculateSignalLength2 (RXChannel_ELE);
		}	
	}
	if ((Changes & RX2_COLL_PIN)!=0)
	{
		if (RX2_COLL)
		{
			RX_raw[1][RXChannel_THR]=TCNT1;
			if (Config.IsESCCalibration==ESCCalibration_ON)
			{M1=1;M2=1;M3=1;M4=1;}
		}
		else
		{
			if ((Config.IsESCCalibration==ESCCalibration_ON) && (IS_TX2_GOOD))
			{M1=0;M2=0;M3=0;M4=0;}
			CalculateSignalLength2 (RXChannel_THR);
			RX2_LastValidSignal_timestamp = TCNT1_X;
			RX_Good = TX2_CONNECTED_ERR;  // CLR bit 5 ---  Status = OK
		}	
	}
	if ((Changes & RX2_YAW_PIN) !=0)
	{
		if (RX2_YAW)
		{
			RX_raw[1][RXChannel_RUD]=TCNT1;
		}
		else
		{
			CalculateSignalLength2 (RXChannel_RUD);
			
		}	
	}
	
}
#endif

//#ifdef BUDDY_CONFIG

#ifdef PRIMARY_INPUT_RX

__attribute__ ((section(".lowtext")))
ISR (RX1_COLL_vect)
{
	if (RX1_COLL)
	{
		RX_raw[0][RXChannel_THR]=TCNT1;

	}
	else
	{
		CalculateSignalLength1(RXChannel_THR);
		
		RX1_LastValidSignal_timestamp = TCNT1_X;
		RX_Good = TX1_CONNECTED_ERR;		// CLR bit 0 ---  Status = OK
	}
	
}



__attribute__ ((section(".lowtext")))
ISR (RX1_ROLL_vect)
{
	if (RX1_ROLL)
	{
		RX_raw[0][RXChannel_AIL]=TCNT1;
	}
	else
	{
		CalculateSignalLength1(RXChannel_AIL);
		
		RX1_LastValidSignal_timestampAux = TCNT1_X;
		RX_Good = TX1_FOUND_ERR;			// CLR bit 1 ---  Status = OK

		
	}
	
}


__attribute__ ((section(".lowtext")))
ISR (RX1_PITCH_vect)
{
	if (RX1_PITCH)
	{
		RX_raw[0][RXChannel_ELE]=TCNT1;
	}
	else
	{
		CalculateSignalLength1(RXChannel_ELE);
	}
}


__attribute__ ((section(".lowtext")))
ISR (RX1_YAW_vect)
{
	if (RX1_YAW)
	{
		RX_raw[0][RXChannel_RUD]=TCNT1;
	}
	else
	{
		CalculateSignalLength1(RXChannel_RUD);
	}
}
//#endif
#endif


__attribute__ ((section(".lowtext")))
ISR (RX_AUX_vect)
{
	// There is only ONE AUX PORT - usually assigned to Secondary RX 
	if (RX_AUX)
	{
		RX_raw[1][RXChannel_AUX]=TCNT1;
		//RX_raw[0][RXChannel_AUX]=TCNT1;
	}
	else
	{
		CalculateSignalLength2(RXChannel_AUX);
		RX_Length[0][RXChannel_AUX]=RX_Length[1][RXChannel_AUX];
	}
}


void RX_Init(void)
{
	
	RX_AUX_DIR   	 	= INPUT;

#ifdef PRIMARY_INPUT_RX

	if (Config.RX_mode==RX_mode_BuddyMode)
	{
	
		ActiveRXIndex=0;

		RX1_ROLL_DIR 		= INPUT;
		RX1_PITCH_DIR 		= INPUT;
		RX1_COLL_DIR   		= INPUT;
		RX1_YAW_DIR   	 	= INPUT;

		// enable interrupts
		EICRA  = _BV(ISC00) | _BV(ISC10) | _BV(ISC20);	// any edge on INT0, INT1 and INT2
		EIMSK  = _BV(INT0)  | _BV(INT1)  | _BV(INT2);	// enable interrupt for INT0, INT1 and INT2
		EIFR   = _BV(INTF0) | _BV(INTF1) | _BV(INTF2);	// clear interrupts
		
		PCICR  |= _BV(PCIE1) | _BV(PCIE3);				// enable PCI1 and PCI3
		PCMSK1 |= _BV(PCINT8);							// enable PCINT8 (AUX) -> PCI1
		PCMSK3 |= _BV(PCINT24);							// enable PCINT24 (THR) -> PCI3
		PCIFR  |= _BV(PCIF1) | _BV(PCIF3);
	}
		
#endif

	
#ifdef SECONDARY_INPUT_RX

	ActiveRXIndex=1;
	
	RX2_ROLL_DIR 		= INPUT;
	RX2_PITCH_DIR 		= INPUT;
	RX2_COLL_DIR   		= INPUT;
	RX2_YAW_DIR   	 	= INPUT;
	
	/*
	PCINT16 - PC0 - OUTPUT 6
	PCINT17 - PC1 - OUTPUT 5 
	PCINT21 - PC5 - OUTPUT 7
	PCINT23 - PC7 - OUTPUT 8 
	*/
	PCICR  |= _BV(PCIE1)   | _BV(PCIE2);														// enable PCI1 and PCI2
	PCMSK1 |= _BV(PCINT8);																		// enable PCINT8 (AUX) -> PCI1
	PCMSK2 |= _BV(PCINT16) | _BV(PCINT17) | _BV(PCINT21) |_BV(PCINT23);							// enable PCINT24 (THR) -> PCI3
	PCIFR  |= _BV(PCIF1)   | _BV(PCIF2);														// clear interrupts


#endif
	

		
	
	RX_Good =TX1_NOT_FOUND;
	RX_Good =TX2_NOT_FOUND;
	
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		RX1_LastValidSignal_timestamp= TCNT1_X;
		RX1_LastValidSignal_timestampAux= TCNT1_X;
	}	
	
	RX2_LastValidSignal_timestamp= RX2_LastValidSignal_timestamp;
	RX2_LastValidSignal_timestampAux= RX2_LastValidSignal_timestampAux;	
}

  uint16_t RX_raw_GetReceiverValues (uint8_t RXIndex, uint8_t Channel)
{
	uint16_t _t;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		_t = RX_Length[RXIndex][Channel];
	}		
	return _t;
}

 int16_t RX_GetReceiverValues (uint8_t RXIndex,uint8_t Channel)
{
	int16_t _t;
	//////if (RX_Good != TX_GOOD) return 0;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		_t = ((int)(RX_Length[RXIndex][Channel]));
	}		
	_t -= Config.RX_Mid[RXIndex][Channel];
	_t /=RX_Div_Factor;
	return _t;
}

/*
// Retrieves Throttle value [0 - 1000].
// IMPORTANT: if throttle signal is accidentally -or using trim- less than Config.RX_Min[RXChannel_THR] then the returned value will be roll back
// to 0xffff range which means an aggressive Quad copter action. we avoid this by using a signed int.
*/
int16_t RX_GetReceiverThrottleValue (uint8_t RXIndex)
{
	
	///if (RX_Good != TX_GOOD) return 0;
	
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		if (RXIndex==0)
		{
			if ( (TCNT1_X - RX1_LastValidSignal_timestamp) > RX_SIGNAL_LOST_DURATION)
			{
				RX_Good =TX1_NOT_FOUND;
				return 0;
			}	
		
			if ( (TCNT1_X - RX1_LastValidSignal_timestampAux) > RX_SIGNAL_LOST_DURATION)
			{
				RX_Good =TX1_DISCONNECTED;
				return 0;
			}
		}	
		if (RXIndex==1)
		{
			if ( (TCNT1_X - RX2_LastValidSignal_timestamp) > RX_SIGNAL_LOST_DURATION)
			{
				RX_Good =TX2_NOT_FOUND;
				return 0;
			}	
		
			if ( (TCNT1_X - RX2_LastValidSignal_timestampAux) > RX_SIGNAL_LOST_DURATION)
			{
				RX_Good =TX2_DISCONNECTED;
				return 0;
			}
		}			
		
		
		iTemp16 = ((int)(RX_Length[RXIndex][RXChannel_THR]));
	}		
	
	iTemp16 -= Config.RX_Min[RXIndex][RXChannel_THR];
	iTemp16 /= RX_Div_Factor;
	 
	return iTemp16;
}
 
void RX_CopyLatestReceiverValues (void)
{
	for (int ch=0;ch<2;++ch)
	{
		for (int i=0;i<RXChannels;++i)
		{
			if (i == RXChannel_THR)
			{
				RX_Latest[ch][i]= RX_GetReceiverThrottleValue(ch);	 
			}
			else
			{
				RX_Latest[ch][i]= RX_GetReceiverValues(ch,i);	 
				if ((RX_Latest[ch][i]<STICK_DEADBAND) && (RX_Latest[ch][i]>-STICK_DEADBAND)) RX_Latest[ch][i]=0;
			}
		}	
	}
				
}


void RX_StickCenterCalibrationInit(uint8_t RXIndex)
{
	for (int i=0; i<RXChannels; ++i)
	{
		RX_MAX_raw[RXIndex][i]=0;
		RX_MIN_raw[RXIndex][i]=0xfffe;
	}
}


void RX_StickCenterCalibration (uint8_t RXIndex)
{
	
	uint16_t tempRX;
	for (int i=0;i<RXChannels;++i)
	{
		tempRX = RX_raw_GetReceiverValues(RXIndex,i);
		if (tempRX!=0)
		{
			
			if ( tempRX > RX_MAX_raw[RXIndex][i]) 
			{
				RX_MAX_raw[RXIndex][i] = tempRX;
			}
			else if (tempRX < RX_MIN_raw[RXIndex][i]) 
			{
				RX_MIN_raw[RXIndex][i] = tempRX;
			}
		}
	}				
}