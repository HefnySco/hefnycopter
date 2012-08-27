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

int16_t	PMW_Pulse_Interval = PWM_LOW_PULSE_INTERVAL;



volatile uint16_t i;
uint16_t MotorStartTCNT, ElapsedTCNT1, ElapsedTCNT2, CurrentTCNT1,CurrentTCNT2;
volatile uint16_t m1,m2,m3,m4;
	
uint16_t tempTCNT1;
uint8_t tempTCNT2;



void Motor_GenerateOutputSignal(void)
{
		// Only enable motors when armed or not connected to the GUI
	
	
	// Make sure we have spent enough time between pulses
	// Also, handle the odd case where the TCNT1 rolls over and TCNT1 < MotorStartTCNT1
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
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
	// TCNT1 tick is 50ns & TCNT2 tick is 3.2us and TCNT2_X ticks every 819.2 us.
	
	
	PMW_Pulse_Interval = (PWM_LOW_PULSE_INTERVAL - ( ElapsedTCNT2 * 409)); // 3276= Tick time of TCNT2_X in us
	while (PMW_Pulse_Interval > 0)
	{
			TCNT2 =0;
			if (PMW_Pulse_Interval>100)
			{   // make big jumps if PWM_Low_Pulse_Interval is high
				// [more accurate to decrease the % between the while loop and the PWM_Low_Pulse_Interval -=80 execution time.]
				while (TCNT2 < 50); //=8us
				PMW_Pulse_Interval -=80;	
			}
			else
			{   //
				while (TCNT2 < 5); //=8us
				PMW_Pulse_Interval -=8;
			}
	}
	
	
	// Set motor limits (0 -> MOTORS_HIGH_VALUE)
	if ( MotorOut1 < 0 ) m1 = 0;
	else if ( MotorOut1 > MOTORS_HIGH_VALUE ) m1 = MOTORS_HIGH_VALUE;
	else m1 = MotorOut1;
	
	if ( MotorOut2 < 0 ) m2 = 0;
	else if ( MotorOut2 > MOTORS_HIGH_VALUE ) m2 = MOTORS_HIGH_VALUE;
	else m2 = MotorOut2;

	if ( MotorOut3 < 0 ) m3 = 0;
	else if ( MotorOut3 > MOTORS_HIGH_VALUE ) m3 = MOTORS_HIGH_VALUE;
	else m3 = MotorOut3;

	if ( MotorOut4 < 0 ) m4 = 0;
	else if ( MotorOut4 > MOTORS_HIGH_VALUE ) m4 = MOTORS_HIGH_VALUE;
	else m4 = MotorOut4;
		
	
	
	// Minimum pulse we want to make is 1ms, max is 2ms
	// So to start, let's make the 1ms base pulse.
	// First, we switch on the motor outputs
	M1 = 1;
	M2 = 1;
	M3 = 1;
	M4 = 1;

	
	// Measure period of ESC rate from here
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      MotorStartTCNT = TCNT2_X;
    }
	
	// Minimum pulse we want to make is 1ms, max is 2ms
	PMW_Pulse_Interval = BASE_PULSE;
	while (PMW_Pulse_Interval > 0)
	{
			TCNT2 =0;
			if (PMW_Pulse_Interval>100)
			{   // make big jumps if PWM_Low_Pulse_Interval is high
				// [more accurate to decrease the % between the while loop and the PWM_Low_Pulse_Interval -=80 execution time.]
				while (TCNT2 < 50); //=8us
				PMW_Pulse_Interval -=80;	
			}
			else
			{   //
				while (TCNT2 < 5); //=8us
				PMW_Pulse_Interval -=8;
			}
	}
		
	
	for (i=0;i<MOTORS_HIGH_VALUE+1;i+=4)			// 1000 gives a max of 2200us 
	{
		//TCNT2=0;
		//while (TCNT2<1);  //1.6us
		
		if (i>=m1) M1 = 0;
		if (i>=m2) M2 = 0;
		if (i>=m3) M3 = 0;
		if (i>=m4) M4 = 0;
		
	}
	
 }
 