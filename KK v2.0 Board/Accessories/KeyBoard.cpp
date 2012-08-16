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



KeyBoard::KeyBoard ()
{
	
}


uint8_t KeyBoard::KeyboardRead()
{
	uint8_t keys = ~KEYBOARD_PIN & ALLKEYS;
	uint8_t newkeys = keys & ~lastKeys;
	lastKeys = keys;
	return newkeys;
}

void KeyBoard::Init (void)
{

	// key pins to input
	KEYBOARD_DDR |= ALLKEYS;

	
	// pull up resistors on
	KEYBOARD_PORT |= ALLKEYS;
	
}

inline uint8_t KeyBoard::KeyboardState()
{
	return lastKeys;
}
	