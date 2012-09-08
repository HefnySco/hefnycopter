/*
 * KeyBoard.cpp
 *
 * Created: 8/16/2012 8:22:51 AM
 *  Author: hefny
 
 Thanks to "Oliver Shulz" schulz@os-x.eu
 */ 


#include <avr/io.h> 
#include "../Include/typedefs.h"
#include "../Include/IO_config.h"
#include "../Include/KeyBoard.h"
#include "../Include/Timer.h"




void KeyBoard_Init (void)
{

	// key pins to input
	KEYBOARD_DDR |= ALLKEYS;

	
	// pull up resistors on
	KEYBOARD_PORT |= ALLKEYS;
	
}

uint16_t TCNT_X_snapshot3=0;
uint8_t Keyboard_Read()
{
	uint8_t keys = ~KEYBOARD_PIN & ALLKEYS;
	uint8_t newkeys = keys & ~lastKeys; // always equal to ZERO  for repeated buttons.
	_keyrepeat = KEYBOARD_NO_REPEAT;
	if (keys !=0) // if any key pressed even repeated.
	{
		if ((lastKeys!=keys))
		{	// Reset Timer if not the same key.
			TCNT_X_snapshot3 = TCNT1_X+300;
		}
		else
		{	// Count to measure the key press duration.
			if ((TCNT_X_snapshot3 < TCNT1_X))
			{
				_keyrepeat = KEYBOARD_REPEAT;
				return keys; // re-press keys and set the repeated flag for more GUI handling.
			}
		}
	}	
	lastKeys = keys;
	return newkeys;
}


inline uint8_t Keyboard_State()
{
	return lastKeys;
}
	