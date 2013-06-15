/*
 * Arming.c
 *
 * Created: 18-Sep-12 6:19:30 AM
 *  Author: M.Hefny
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <avr/wdt.h>
#include <util/atomic.h>

#include "Include/typedefs.h"
#include "Include/GlobalValues.h"
#include "Include/Arming.h"
#include "Include/Menu_Text.h"
#include "Include/Beeper.h"
#include "Include/Menu_Screen.h"
#include "Include/Receiver.h"

void Disarm (void)
{
	IsArmed = false;
	//LED_FlashOrangeLED (LED_SHORT_TOGGLE,4);
	LED_Orange = OFF;
	
	TCNT1_X_snapshot1 =0; // reset timer
	Beeper_Beep(BEEP_SHORT,2);				
	Menu_LoadPage (PAGE_HOME);
}


void Arm (void)
{
	TCNT1_X_snapshot1 =0; // reset timer
	
	
	for (int channel=0;channel< SENSORS_IMU;++channel)
	{
		uint16_t temp = ADCPort_Get(SensorsIndex[channel]);
		if ((GyroLowLimit > temp) || (temp > GyroHighLimit))
		{
			Disarm();
			Menu_LoadPage(PAGE_SENSOR_TEST);
			return;
		}
	}	
	
	if (
		((Config.RX_mode==RX_mode_BuddyMode) && (!IS_TX1_GOOD)) // in Buddy mode you cannot arm is there is no signal from TX1
		||
		((Config.FrameType == FRAMETYPE_TRICOPTER) && (Config.BoardOrientationMode==QuadFlyingMode_X))  // Invalid configuration.
		||
		(page != PAGE_HOME) // suggested by samog so that pilot can see final flying config.
		)
		{
			Disarm();
			return ;
		}			
		
	
		
	IsArmed = true;
	LED_FlashOrangeLED (LED_SHORT_TOGGLE,4);
	LED_Orange = ON;
	Beeper_Beep(BEEP_LONG,1);				
	
	
	TCNT_X_snapshotAutoDisarm=0;				
	Menu_LoadPage (PAGE_HOME_ARMED);
	
	PID_GyroTerms[PITCH_INDEX].I=0;
	PID_GyroTerms[ROLL_INDEX].I=0;
	PID_GyroTerms[YAW_INDEX].I=0;
	PID_AccTerms [PITCH_INDEX].I=0;
	PID_AccTerms [ROLL_INDEX].I=0;
	PID_AccTerms [Z_INDEX].I=0;
		
}
