/*
 * Init.c
 *
 * Created: 24-Jul-12 9:54:30 PM
 *  Author: M.Hefny
 */ 




/* ----------- Main Code -----------  */

#include <avr/io.h>  
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include <avr/interrupt.h> 

#include "typedefs.h"
#include "io_cfg.h"
#include "Commons.h"
#include "Init.h"



void InitIO (void)
{
	
	RX_ROLL_DIR 		= INPUT;
	RX_PITCH_DIR 		= INPUT;
	RX_COLL_DIR   		= INPUT;
	RX_YAW_DIR   	 	= INPUT;

	GYRO_YAW_DIR 	 	= INPUT;
	GYRO_PITCH_DIR 	 	= INPUT;
	GYRO_ROLL_DIR  		= INPUT;
	GAIN_YAW_DIR 	 	= INPUT;
	GAIN_PITCH_DIR		= INPUT;
	GAIN_ROLL_DIR  		= INPUT;

	M1_DIR 				= OUTPUT;
	M2_DIR 				= OUTPUT;
	M3_DIR 			 	= OUTPUT;
	M4_DIR 			 	= OUTPUT;

	LED_DIR 			= OUTPUT;

	LED			= 0;
	RX_ROLL 	= 0;
	RX_PITCH 	= 0;
	RX_COLL  	= 0;
	RX_YAW   	= 0;
	
	
	
	// Interrupts
	// pin change interrupt enables
	PCICR  |= (1 << PCIE0);			// PCINT0..7		
	PCICR  |= (1 << PCIE2);			// PCINT16..23

	// pin change masks
	PCMSK0 |= (1 << PCINT7);		// PB7
	PCMSK2 |= (1 << PCINT17);		// PD1
	// external interrupts
	EICRA   = (1 << ISC00) | (1 << ISC10);		// Any change INT0, INT1
	EIMSK   = (1 << INT0)  | (1 << INT1);		// External Interrupt Mask Register
	EIFR   |= (1 << INTF0) | (1 << INTF1);
	
}


void InitTimers (void)
{
	
	// Timer 0 is a count time timer using OCR0A and OCR0A_X
	// timer0 (8bit) - run @ 8MHz / 64 = 125 KHz = 8us
	// used to control ESC/servo pulse length
	TCCR0A = (1 << WGM01);					// Mode = CTC
    TCCR0B = (1 << CS02); // | (1 << CS00);		//| (1 << CS00); 
	//OCR0A = 255;							// 8us * 255 = 
	//OCR0A_X=2;
	//TIMSK0  |= (1 << OCIE0A); 				// Enable Interrupt TimerCounter0 Compare Match A (SIG_OUTPUT_COMPARE0A)
	
	// timer1 (16bit) - run @ 1Mhz step = 1 us
	// used to measure Rx Signals & control ESC/servo output rate
	TCCR1A  = 0;
	TCCR1B  = (1 << CS11);
	TIMSK1 |= (1 << TOIE1); // Enable overflow interrupt 
	TCNT1   = 0;		// reset counter
	TCNT1_X = 0;		// This value overflow every 4294.967296 sec [1.19 hr], and tick every 0.065536 sec

		
	// timer2 8bit - run @ 8MHz / 8 = 250 KHz = 4us
	// and Stick-Arming
	TCCR2A  = (1 << WGM22);	
	TCCR2B  = (1 << CS21) | (1 << CS20);	
	TIMSK2 |= (1 << TOIE2); // Enable overflow interrupt 
	TIFR2   = 0;
	TCNT2   = 0;		// reset counter
	TCNT2_X = 0;		// this value overflow every 4us x 0xffff = 0.262144 sec, and tick every 0.001024 sec
}