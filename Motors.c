/*
 * Motors.c
 *
 * Created: 22-Jul-12 9:33:00 PM
 *  Author: M.Hefny
 */ 





/* ----------- Main Code -----------  */

#include <avr/io.h>  
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include <avr/interrupt.h> 
#include <avr/eeprom.h>


#include "typedefs.h"
#include "io_cfg.h"
#include "Commons.h"
#include "motors.h"

int16_t	PWM_Low_Pulse_Interval = PWM_LOW_PULSE_INTERVAL;



volatile uint8_t i;
volatile static uint16_t MotorStartTCNT1, ElapsedTCNT1, CurrentTCNT1;
volatile uint8_t m1,m2,m3,m4;
	


void output_motor_ppm(void)
{
	// Make sure we have spent enough time between pulses
	// Also, handle the odd case where the TCNT1 rolls over and TCNT1 < MotorStartTCNT1
	
	
	CurrentTCNT1 = TCNT1;
	if (CurrentTCNT1 > MotorStartTCNT1) ElapsedTCNT1 = CurrentTCNT1 - MotorStartTCNT1;
	else ElapsedTCNT1 = (0xffff - MotorStartTCNT1) + CurrentTCNT1;
		
	// If period less than 1/ESC_RATE, pad it out.
	PWM_Low_Pulse_Interval = (PWM_LOW_PULSE_INTERVAL - ElapsedTCNT1) / 8;
	
	if (PWM_Low_Pulse_Interval > 0)
	{
		TIFR2 &= ~(1 << TOV2);		// Clear overflow
		TCNT2 = 0;					// Reset counter
		
		for (i=0;i<PWM_Low_Pulse_Interval;i++)
		{
			while (TCNT2 < 128);   // TCNT2=128 ==> 512 micro seconds [@ 8 MHz / 32].
			TCNT2-=128;
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
	MotorStartTCNT1 = TCNT1;

	
	// Create the base pulse of 1120us
	TIFR2 &= ~(1 << TOV2);		// Clear overflow
	TCNT2 = 0;					// Reset counter
	
	for (i=0;i<BASE_PULSE;i++)	// BASE_PULSE * 8us = 1ms
	{
		while (TCNT2 < 4);		// [8MHz / 32] x 4 = 16us
		TCNT2 = 0;
	}
	
	
	TCNT2 = 0;					// Reset counter again
	// Now switch off the pulses as required
	// 1120us to 1920us = 800us / 4us = 200 steps
	// Motors 0->200, 1120->1920 us
	for (i=0;i<MOTORS_HIGH_VALUE+20;i++)			// 220 gives a max of 2000us (1120 + (220 * 4us)) - TWEAK THIS
	{
		// TIP: motor accuracy in SW is [0-200] while we can go from [0-800] we need to enhance SW range to get better accuracy
		// TIP2: we also need to speed up TCNT2 to achieve better accuracy.
		while (TCNT2 < 1);		// 8MHz * 32 = 4us
		TCNT2 = 0;

		// stop motor when its on-duration is over
		if (i==m1) M1 = 0;		
		if (i==m2) M2 = 0;
		if (i==m3) M3 = 0;
		if (i==m4) M4 = 0;
	} 
	
	/*
	//OCR0A = 8 * BASE_PULSE;
	OCR0A +=4;  //first loop in the output_motor_ppm2()  
	pmm_out_step=1;
	//pMotorFunction = output_motor_ppm2;
	output_motor_ppm2();
	*/

}
	