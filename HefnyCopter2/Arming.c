/*
 * Arming.c
 *
 * Created: 18-Sep-12 6:19:30 AM
 *  Author: M.Hefny
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <avr/wdt.h>
#include <util/atomic.h>

#include "Include/typedefs.h"
#include "Include/GlobalValues.h"
#include "Include/Arming.h"
#include "Include/Menu_Text.h"


void Disarm (void)
{
	IsArmed = false;
	LED_Orange = OFF;
	LED_FlashOrangeLED (LED_LONG_TOGGLE,4);

	TCNT1_X_snapshot1 =0; // reset timer
					
	Menu_LoadPage (PAGE_HOME);
}


void Arm (void)
{
	IsArmed = true;
	LED_Orange = ON;
	LED_FlashOrangeLED (LED_LONG_TOGGLE,4);

	TCNT1_X_snapshot1 =0; // reset timer
	TCNT_X_snapshotAutoDisarm=0;				
	Menu_LoadPage (PAGE_HOME_ARMED);
}
