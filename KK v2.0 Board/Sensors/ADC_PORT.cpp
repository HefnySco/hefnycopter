/*
 * ADC.cpp
 *
 * Created: 8/14/2012 2:30:13 AM
 *  Author: hefny
 */ 


#include <avr/io.h> 


#include "../Include/typedefs.h"
#include "../Include/ADC_PORT.h"
#include "../Include/IO_config.h"

ADC_PORT::ADC_PORT ()
{
	
}


void ADC_PORT::Init ()
{
			  //76543210	
	DIDR0 	= 0b11111111;	// turn off digital inputs on port A
	
}