/*
 * Motor.cpp
 *
 * Created: 8/13/2012 8:46:17 PM
 *  Author: hefny
 */ 


/* ----------- Main Code -----------  */

#include <avr/io.h>  
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/interrupt.h> 
#include <avr/eeprom.h>


#include "Include/typedefs.h"
#include "Include/IO_config.h"
#include "Include/GlobalValues.h"
#include "Include/Motor.h"

#include "Include/Menu.h"

int16_t	PMW_Pulse_Interval ;



//volatile uint16_t i;

volatile uint16_t m1,m2,m3,m4;
	
uint16_t tempTCNT1;
uint8_t tempTCNT2;

static  uint8_t times=0;


void Motor_GenerateOutputSignal(void)
{
	int i;
	// Only enable motors when armed or not connected to the GUI
	
	
	// Make sure we have spent enough time between pulses
	// Also, handle the odd case where the TCNT2_X rolls over and TCNT2_X < ElapsedTCNT2
	ATOMIC_BLOCK(ATOMIC_FORCEON)
   {
      CurrentTCNT2 = TCNT2_X;
   }

   if (CurrentTCNT2 >= MotorStartTCNT) 
   {
		ElapsedTCNT2 = CurrentTCNT2 - MotorStartTCNT;
   }
   else 
   {
		ElapsedTCNT2 = (0xffff - MotorStartTCNT) + CurrentTCNT2;
   }		
	
	// If period less than 1/ESC_RATE, pad it out.
	// check Timer.c for timing table
	
	//interval in us
	PMW_Pulse_Interval = (PWM_LOW_PULSE_INTERVAL - ( ElapsedTCNT2 * 409)); // 409 = OV time of TCNT2
	while (PMW_Pulse_Interval > 0)
	{
		//LED_Orange=~LED_Orange;
			TCNT2 =0;
			if (PMW_Pulse_Interval>100)
			{   // make big jumps if PWM_Low_Pulse_Interval is high
				// [more accurate to decrease the % between the while loop and the PWM_Low_Pulse_Interval -=80 execution time.]
				while (TCNT2 < 40); //=8us 1.50
				PMW_Pulse_Interval -=80;	
			}
			else
			{   //
				///////////while (TCNT2 < 5); //=8us
				PMW_Pulse_Interval -=1;
			}
			
	}
	
	times+=1;
	
	// Set motor limits (0 -> MOTORS_HIGH_VALUE)
	if ( MotorOut[0] < 0 ) m1 = 0;
	else if ( MotorOut[0] > MOTORS_HIGH_VALUE ) m1 = MOTORS_HIGH_VALUE;
	else m1 = MotorOut[0];
	
	if ( MotorOut[1] < 0 ) m2 = 0;
	else if ( MotorOut[1] > MOTORS_HIGH_VALUE ) m2 = MOTORS_HIGH_VALUE;
	else m2 = MotorOut[1];

	if ( MotorOut[2] < 0 ) m3 = 0;
	else if ( MotorOut[2] > MOTORS_HIGH_VALUE ) m3 = MOTORS_HIGH_VALUE;
	else m3 = MotorOut[2];

	if ( MotorOut[3] < 0 ) m4 = 0;
	else if ( MotorOut[3] > MOTORS_HIGH_VALUE ) m4 = MOTORS_HIGH_VALUE;
	else m4 = MotorOut[3];
		
	
	
	// Minimum pulse we want to make is 1ms, max is 2ms
	// So to start, let's make the 1ms base pulse.
	// First, we switch on the motor outputs
	M1 = 1;
	M2 = 1;
	M3 = 1;
	if (Config.FrameType== FRAMETYPE_QUADCOPTER) 
	{
		M4 =1;
	}
	else if (Config.FrameType== FRAMETYPE_TRICOPTER)
	{
		if (times==4)
		{
			M4 = 1;		// time to update servo
			//UpdateServo = FALSE;
			times=0;	// reset time divider
		} // else keep it zero.			
	} 
	
	
	// Minimum pulse width we want to make is 1ms, max is 2ms
	PMW_Pulse_Interval = BASE_PULSE + Config.ThrottleMin;
	while (PMW_Pulse_Interval > 0)
	{
			TCNT2 =0;
			if (PMW_Pulse_Interval>100)
			{   // make big jumps if PWM_Low_Pulse_Interval is high
				// [more accurate to decrease the % between the while loop and the PWM_Low_Pulse_Interval -=80 execution time.]
				while (TCNT2 < 45); //=8us //1. 50
				PMW_Pulse_Interval -=80;	
			}
			else
			{   //
				//////while (TCNT2 < 5); //=8us
				PMW_Pulse_Interval -=1;
			}
	}
		
	
	for (i=0;i<MOTORS_HIGH_VALUE+4;i+=2)			// 1000 gives a max of 2200us 
	{
		/*
		MOTORS_HIGH_VALUE+4
		in order to guarantee that all Ms are zeros when getting out of this loop.
		*/
		
		if (i>=m1) M1 = 0;
		if (i>=m2) M2 = 0;
		if (i>=m3) M3 = 0;
		if (i>=m4) M4 = 0;
	}
	// Measure period of ESC rate from here
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
      MotorStartTCNT = TCNT2_X;
    }
	
	
 }
 