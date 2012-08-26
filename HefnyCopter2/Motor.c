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

int16_t	PWM_Low_Pulse_Interval = PWM_LOW_PULSE_INTERVAL;



volatile uint8_t i;
volatile static uint16_t MotorStartTCNT2, ElapsedTCNT1, CurrentTCNT1,CurrentTCNT2;
volatile uint8_t m1,m2,m3,m4;
	
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

   if (CurrentTCNT1 > MotorStartTCNT1) 
   {
		ElapsedTCNT2 = CurrentTCNT2 - MotorStartTCNT2;
   }
   else 
   {
		ElapsedTCNT2 = (0xffff - MotorStartTCNT2) + CurrentTCNT2;
   }		
	
	
	
	// If period less than 1/ESC_RATE, pad it out.
	// TCNT1 tick is 50ns & TCNT2 tick is 3.2us and TCNT2_X ticks every 8192 us.
	PWM_Low_Pulse_Interval = (PWM_LOW_PULSE_INTERVAL - ElapsedTCNT2)  ; //>> 3  ;
	while (PWM_Low_Pulse_Interval > 0)
	{
		TCNT2=0;
		tempTCNT2 = TCNT2 + 4; 	
		//LED =~LED;
		while (TCNT2 < tempTCNT2);		// 8MHz * 64 = 8us
		PWM_Low_Pulse_Interval -=1;
		
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
		
	
	
	// T0 = 8 bit @ 8MHz, so 1 count per 125ns, max of 32us
	// T1 = 16 bit @ 1MHz, so 1 count per us, max of 65,539us or 65.5ms
	// CurrentTCNT1 = TCNT1; // Snapshot

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
      MotorStartTCNT1 = TCNT1;
   }
	
	
	// Create the base pulse of 1120us
	//TIFR2 &= ~(1 << TOV2);		// Clear overflow
	//TCNT2 = 0;					// Reset counter
	//tempTCNT1 = TCNT1 + 16; 
	TCNT2 = 0;
	tempTCNT2 = TCNT2 + 4; 
	for (i=0;i<BASE_PULSE;i++)	// BASE_PULSE * 8us = 1ms
	{
		while (TCNT2 < tempTCNT2);		// 8MHz * 64 = 8us
		TCNT2=0;
		tempTCNT2 = TCNT2 + 4; 	
		////while (TCNT1 < tempTCNT1);
		////tempTCNT1 = TCNT1 + 16;
	}
	
	
	//tempTCNT1 = TCNT1 + 4;
	// Now switch off the pulses as required
	// 1120us to 1920us = 800us / 4us = 200 steps
	// Motors 0->200, 1120->1920 us
	
	//TCNT2 = 0;
	//tempTCNT2 = TCNT2 + 1; 
	for (i=0;i<MOTORS_HIGH_VALUE+1;i++)			// 220 gives a max of 2000us (1120 + (220 * 4us)) - TWEAK THIS
	{
		//while (TCNT2 < tempTCNT2);

		// NO CODE SHOULD BE ADDED HERE . IT IS ECAXCTLY 4 us loop
		//asm ("nop");
		//asm ("nop");
		//asm ("nop");
		//asm ("nop");
		
		if (i==m1) M1 = 0;
		if (i==m2) M2 = 0;
		if (i==m3) M3 = 0;
		if (i==m4) M4 = 0;
		//TCNT2 = 0;
		//tempTCNT2 = TCNT2 + 1; 
	} 

}
