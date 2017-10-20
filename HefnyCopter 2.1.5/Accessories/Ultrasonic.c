/*
 * Ultrasonic.cpp - Library for HC-SR04 Ultrasonic Sensing Module.
 *
 * With ideas from:
 *   Created by ITead studio. Apr 20, 2010.
 *   iteadstudio.com
 *
 * SVN Keywords
 * ----------------------------------
 * $Author: cnobile $
 * $Date: 2011-10-08 21:07:42 -0400 (Sat, 08 Oct 2011) $
 * $Revision: 35 $
 * ----------------------------------
 *
 * Centimeters Divisor
 * =========== =======
 *  15.875     26.9029
 *  46.355     27.6233
 *  92.075     28.1949
 * 137.795     28.4717
 * 183.515     28.5584
 * 229.235     28.5936
 * 274.955     28.7194
 *
 * Inches      Divisor
 * ======      =======
 *   6.25      68.3333
 *  18.25      70.1633
 *  36.25      71.6152
 *  54.25      72.3182
 *  72.25      72.5384
 *  90.25      72.6277
 * 108.25      72.9473
 */

#include <stdlib.h>
#include <string.h>


#include "../Include/typedefs.h"
#include "../Include/IO_config.h"
#include "../Include/GlobalValues.h"
#include "..\Include\Ultrasonic.h"


void Ultrasonic_Init ()
{
	RX_SONAR_TRIGGER_DIR = OUTPUT;
	RX_SONAR_TRIGGER=HIGH;   // Active LOW
	RX_SONAR_DIR = INPUT;
	
	// enable interrupts
	EICRA  = _BV(ISC20);	// any edge on INT2
	EIMSK  = _BV(INT2);	// enable interrupt for INT0, INT1 and INT2
	EIFR   = _BV(INTF2);	// clear interrupts
		
	
}
