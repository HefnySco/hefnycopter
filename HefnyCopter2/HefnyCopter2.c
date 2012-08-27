/*
 * HefnyCopter2.cpp
 *
 * Created: 8/13/2012 6:30:08 PM
 *  Author: hefny
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <avr/wdt.h>
#include <util/atomic.h>

#include "Include/GlobalValues.h"
#include "Include/HefnyCopter2.h"
#include "Include/IO_config.h"
#include "Include/LED.h"
#include "Include/Beeper.h"
#include "Include/Sensors.h"
#include "Include/ADC_PORT.h"
#include "Include/GlobalValues.h"
#include "Include/LCD.h"
#include "Include/Motor.h"
#include "Include/KeyBoard.h"
#include "Include/Menu.h"
#include "Include/Timer.h"
#include "Include/Receiver.h"
#include "Include/eepROM.h"
#include "Include/fonts.h"
#include "Include/Menu_Text.h"

/*

Quad
                    M1 CW
                     |
                     |
                     |
                   +---+
		 CCW M2----|   |----M3 CCW
                   +---+
                     |
                     |
                     |
                   M4 CW
				   
				   
Quad-X
       
           M1 CW       M3 CCW
			  \          /
               \        / 
                \ ---  /
                 |    |
                / ---  \
               /        \ 
			  /          \ 
          M2 CCW        M4 CW
		  
		  NOTE: X-QUAD motors order are different from many other code on Internet such as XXController & QuadControllerV#_#

*/

static const prog_char versionNum[] = "Version 0.1";
static const prog_char versionAuthor[] = "HefnyCopter2";



void Setup (void)
{
	RX_Init();


	// Motors
	M1_DIR = OUTPUT;
	M2_DIR = OUTPUT;
	M3_DIR = OUTPUT;
	M4_DIR = OUTPUT;
	M5_DIR = OUTPUT;
	M6_DIR = OUTPUT;
	M7_DIR = OUTPUT;
	M8_DIR = OUTPUT;
	
	Buzzer_DIR = OUTPUT;
	LED_Orange_DIR = OUTPUT;
	
	
	// Sensors
	V_BAT  = INPUT;
	
	Initial_EEPROM_Config_Load();
	
	// Timers
	TCCR1A = 0;	//Set timer 1 to run at 2.5MHz
	TCCR1B = 0;
	TCCR1C = 0;
	
	
	// enable interrupts
	EICRA  = _BV(ISC00) | _BV(ISC10) | _BV(ISC20);	// any edge on INT0, INT1 and INT2
	EIMSK  = _BV(INT0)  | _BV(INT1)  | _BV(INT2);	// enable interrupt for INT0, INT1 and INT2
	EIFR   = _BV(INTF0) | _BV(INTF1) | _BV(INTF2);	// clear interrupts
		
	PCICR |= _BV(PCIE1) | _BV(PCIE3);				// enable PCI1 and PCI3
	PCMSK1 = _BV(PCINT8);							// enable PCINT8 (AUX) -> PCI1
	PCMSK3 = _BV(PCINT24);							// enable PCINT24 (THR) -> PCI3
	PCIFR  = _BV(PCIF1) | _BV(PCIF3);				// clear interrupts

	ADCPort_Init();
	Sensors_Init();
	KeyBoard_Init();
	Timer_Init();
	Menu_MenuInit();
	
		
	LCD_Init();
	LCD_Clear();
	
	LCD_SetPos(1, 0);
	LCD_WriteString_P(versionNum);
	LCD_SetPos(3, 0);
	LCD_WriteString_P(versionAuthor);
	
	sei();
	
	delay_ms(20);
    
}


int main(void)
{
	
	Setup();
	
	 
	// If not calibrated then dont loop in the main loop to menu or motor actions betcause of RX channel positions.
	while ((!(Config.IsCalibrated & CALIBRATED_SENSOR)) || (!(Config.IsCalibrated & CALIBRATED_Stick)))
	{
		Loop();
	}
	
	while(1)
    {
    	MainLoop();
    }
}




BOOL bXQuadMode;


/*
	We are in this loop because the system is not calibrated.
	This is the minimum functionality.
*/
void Loop(void)
{
	
	if (TCNT2_X_snapshot2==0) TCNT2_X_snapshot2 = TCNT2_X;
	if ( (TCNT2_X- TCNT2_X_snapshot2) > LCD_RefreashRate )  
	{
		Menu_MenuShow();	
		TCNT2_X_snapshot2=0;
	}		

}


/*
	This is the main loop of the application.
*/
void MainLoop(void)
{
	
	RX_CopyLatestReceiverValues();
		
	bResetTCNR1_X = true;
	
	// HINT: you can try to skip this if flying to save time for more useful tasks as user cannot access menu when flying
	if (TCNT2_X_snapshot2==0) TCNT2_X_snapshot2 = TCNT1_X;
	else if ( (!IsArmed) && (TCNT1_X- TCNT2_X_snapshot2) > 20 )  // TCNT1_X ticks in 3.2768us
	{
		Menu_MenuShow();	
		TCNT2_X_snapshot2=0;
	}		
	
	if (RX_Latest[RXChannel_THR] < STICKThrottle_ARMING) 
	{	// Throttle is LOW
		// Here you can add code without caring about delays. As there quad is already off and on land.
		// here we test different positions of sticks to enable arm/disarm, Quad/X-Quad
		
		
		if (TCNT1_X_snapshot1==0)  TCNT1_X_snapshot1 = TCNT1_X; // start counting
		
		/////ReadGainValues(); // keep reading values of POTS here. as we can change the value while quad is armed. but sure it is on land and motors are off.
		// DisArm Check
		if ((IsArmed == true) && (RX_Latest[RXChannel_RUD] < STICK_RIGHT))
		{
			bResetTCNR1_X  = false;
			//if (TCNT1_X_snapshot1==0)  TCNT1_X_snapshot1 = TCNT1_X; // start counting
			if ( (TCNT1_X - TCNT1_X_snapshot1) > STICKPOSITION_LONG_TIME )
			{
				IsArmed = false;
				LED_Orange = OFF;
				LED_FlashOrangeLED (LED_LONG_TOGGLE,4);
				TCNT1_X_snapshot1 =0; // reset timer
			}
		}
		
		// Arm Check
		if ((IsArmed == false) && (RX_Latest[RXChannel_RUD] > STICK_LEFT))
		{
			bResetTCNR1_X = false;
			//if (TCNT1_X_snapshot1==0)  TCNT1_X_snapshot1 = TCNT1_X; // start counting
			if ( (TCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_LONG_TIME )
			{
				IsArmed = true;
				
				LCD_Clear();
				LCD_SelectFont (&font12x16);
				LCD_SetPos(3, 30);
				LCD_WriteString_P(strARMED);
				LCD_SelectFont (NULL);
				Menu_MenuInit();
				
				LED_Orange = ON;
				LED_FlashOrangeLED (LED_LONG_TOGGLE,4);
				///CalibrateGyros();
				///LED_FlashOrangeLED (LED_SHORT_TOGGLE,4);
				TCNT1_X_snapshot1 =0; // reset timer
			}		
		}
		
		// Change Mode
		if (IsArmed==false)
		{	//set modes Quad , X-Quad
		
			if (RX_Latest[RXChannel_AIL]  < STICK_RIGHT)
			{// X-QUAD MODE
				bResetTCNR1_X = false;
				//if (TCNT1_X_snapshot1==0)  TCNT1_X_snapshot1 = TCNT1_X; // start counting
				if ( (TCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_LONG_TIME )
				{
					bXQuadMode = true;
					LED_FlashOrangeLED (LED_LONG_TOGGLE,8);
					TCNT1_X_snapshot1 =0; // reset timer
				}
			}			
			else  if ((RX_Latest[RXChannel_AIL]  > STICK_LEFT))
			{	// QUAD COPTER MODE
				bResetTCNR1_X = false;
				//if (TCNT1_X_snapshot1==0)  TCNT1_X_snapshot1 = TCNT1_X; // start counting
				if ( (TCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_LONG_TIME )
				{
					bXQuadMode = false;
					LED_FlashOrangeLED (LED_LONG_TOGGLE,4);
					TCNT1_X_snapshot1 =ON; // reset timer
				}		
			
			} 
			
			
		}
	
		// Stop motors if Throttle Stick is less than minimum.
		MotorOut1 = 0;
		MotorOut2 = 0;
		MotorOut3 = 0;
		MotorOut4 = 0;
	}
	else
	{	// Throttle stick is NOT Down
		
		if (IsArmed==false)
		{  // However we are still DisArmed
			MotorOut1 = 0;
			MotorOut2 = 0;
			MotorOut3 = 0;
			MotorOut4 = 0;
			
			
			///////////////// Sticks as Keyboard --- we are already disarmed to reach here.
			if ((Config.IsCalibrated & CALIBRATED_SENSOR) && (Config.IsCalibrated & CALIBRATED_Stick) && RX_Latest[RXChannel_THR] > STICKThrottle_ARMING)
			{ // if Throttle is high and stick are calibrated
		
				if (TCNT1_X_snapshot1==0)  TCNT1_X_snapshot1 = TCNT1_X; // start counting
				
	 			if ((RX_Latest[RXChannel_ELE]) > STICK_LEFT) 
				{
					bResetTCNR1_X = false;
					if ( (TCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_SHORT_TIME )
					{
						_TXKeys = KEY_3;
						TCNT1_X_snapshot1 =0; // reset timer
					}		
				
				}
				else if ((RX_Latest[RXChannel_ELE]) < STICK_RIGHT) 
				{
					bResetTCNR1_X = false;
					if ( (TCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_SHORT_TIME )
					{
						_TXKeys = KEY_2;
						TCNT1_X_snapshot1 =0; // reset timer
					}		
				
				}		 	
				
				if ((RX_Latest[RXChannel_AIL]) > STICK_LEFT) 
				{
					bResetTCNR1_X = false;
					if ( (TCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_SHORT_TIME )
					{
						_TXKeys = KEY_4;
						TCNT1_X_snapshot1 =0; // reset timer
					}		
				
				}
				else if ((RX_Latest[RXChannel_AIL]) < STICK_RIGHT) 
				{
					bResetTCNR1_X = false;
					if ( (TCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_SHORT_TIME )
					{
						_TXKeys = KEY_1;
						TCNT1_X_snapshot1 =0; // reset timer
					}		
				
				}		 		
			}	
			///////////////// EOF Sticks as Keyboard
	
		}
		else
		{	// Armed & Throttle Stick > MIN . . . We should Fly now.
			if (RX_Latest[RXChannel_THR] <( STICKThrottle_ARMING - 20)) // calibrate again before leaving ground to average vibrations.
			{
				//CalibrateGyros();
			}
				
			
			MotorOut1 = RX_Latest[RXChannel_THR];
			MotorOut2 = RX_Latest[RXChannel_THR];
			MotorOut3 = RX_Latest[RXChannel_THR];
			MotorOut4 = RX_Latest[RXChannel_THR];		
	
	
			///////*
			//////*
			//////*	Stabilization Logic.
			//////*	The logic is independent of Quad configuration 
			//////*/
			//////
			//////ReadGyros();
			//////ReadGainValues();
			//////
				//////
			//////// LIMIT GYRO
			///////*	if (gyroADC[PITCH]> MAX_GYRO_VALUE)		gyroADC[PITCH] = MAX_GYRO_VALUE;
			//////if (gyroADC[PITCH]< -MAX_GYRO_VALUE)	gyroADC[PITCH] = -MAX_GYRO_VALUE;
			//////if (gyroADC[ROLL]> MAX_GYRO_VALUE)		gyroADC[ROLL] = MAX_GYRO_VALUE;
			//////if (gyroADC[ROLL]< -MAX_GYRO_VALUE)		gyroADC[ROLL] = -MAX_GYRO_VALUE;
			//////if (gyroADC[YAW]> MAX_GYRO_VALUE)		gyroADC[YAW] = MAX_GYRO_VALUE;
			//////if (gyroADC[YAW]< -MAX_GYRO_VALUE)		gyroADC[YAW] = -MAX_GYRO_VALUE;
			//////*/
				//////// calculate PITCH
				//////cPITCH   = gyroADC[PITCH];
				//////cPITCH  *= (GainInADC[PITCH]); //   * PITCH_GAIN_MULTIPLIER);
				//////cPITCH  /= ADC_GAIN_DIVIDER;
				//////
				//////// calculate ROLL
				//////cROLL    = gyroADC[ROLL];							
				//////cROLL   *= (GainInADC[PITCH]); // /*GainInADC[ROLL]*/  * ROLL_GAIN_MULTIPLIER);		
				//////cROLL   /= ADC_GAIN_DIVIDER;	
				//////
				//////// calculate YAW
				//////cYAW     = gyroADC[YAW]; 
				//////cYAW	 *= GainInADC[YAW] ; //* YAW_GAIN_MULTIPLIER; 
				//////cYAW    /= ADC_GAIN_DIVIDER;
				//////cYAW	+= ((GainInADC[ROLL] - MID_POT) >> 5);
			//////
			//////if (cPITCH > MAX_GYRO_RESPONSE_VALUE)		cPITCH = MAX_GYRO_RESPONSE_VALUE;
			//////if (cPITCH < -MAX_GYRO_RESPONSE_VALUE)		cPITCH = -MAX_GYRO_RESPONSE_VALUE;
			//////if (cROLL  > MAX_GYRO_RESPONSE_VALUE)		cROLL  = MAX_GYRO_RESPONSE_VALUE;
			//////if (cROLL  < -MAX_GYRO_RESPONSE_VALUE)		cROLL  = -MAX_GYRO_RESPONSE_VALUE;
			//////if (cYAW   > MAX_GYRO_RESPONSE_VALUE)		cYAW   = MAX_GYRO_RESPONSE_VALUE;
			//////if (cYAW   < -MAX_GYRO_RESPONSE_VALUE)		cYAW   = -MAX_GYRO_RESPONSE_VALUE;
				//////
					//////
				//////// Add ROLL [chk reverse - add to RX - update motors]
				//////if (Config.RollGyroDirection == GYRO_REVERSED) cROLL = cROLL * (-1);	
				//////MotorOut2 -= cROLL;
				//////MotorOut3 += cROLL;
		//////
				//////// Add PITCH [chk reverse - add to RX - update motors]
				//////if (Config.PitchGyroDirection == GYRO_REVERSED) cPITCH = cPITCH * (-1);	
				//////MotorOut1 -= cPITCH;
				//////MotorOut4 += cPITCH;
		//////
				//////// Add YAW [chk reverse - add to RX - update motors]
				//////if (Config.YawGyroDirection== GYRO_REVERSED) cYAW = cYAW * (-1);	
				//////MotorOut1 += cYAW;
				//////MotorOut2 -= cYAW;
				//////MotorOut3 -= cYAW;
				//////MotorOut4 += cYAW;
				//////
				
			
			/*
			*
			*	Pilot Control Logic.
			*	
			*/
	
			//if (bXQuadMode==true)
			//{
							//
				//MotorOut1 += RX[RXChannel_AIL] ;
				//MotorOut2 += RX[RXChannel_AIL] ;
				//MotorOut3 -= RX[RXChannel_AIL] ;
				//MotorOut4 -= RX[RXChannel_AIL] ;
				//
				//MotorOut1 += RX[RXChannel_ELE];
				//MotorOut2 -= RX[RXChannel_ELE];
				//MotorOut3 += RX[RXChannel_ELE];
				//MotorOut4 -= RX[RXChannel_ELE];
				//
				//MotorOut1 -= RX[RXChannel_RUD];
				//MotorOut2 += RX[RXChannel_RUD];
				//MotorOut3 += RX[RXChannel_RUD];
				//MotorOut4 -= RX[RXChannel_RUD];
			//}
			//else
			//{
				//MotorOut2 += RX[RXChannel_AIL] ;
				//MotorOut3 -= RX[RXChannel_AIL] ;
				//
				//MotorOut1 += RX[RXChannel_ELE] ;
				//MotorOut4 -= RX[RXChannel_ELE] ;
		//
				//MotorOut1 -= RX[RXChannel_RUD] ;
				//MotorOut2 += RX[RXChannel_RUD] ;
				//MotorOut3 += RX[RXChannel_RUD] ;
				//MotorOut4 -= RX[RXChannel_RUD] ;
			//}
			
			
			// Save motors from turning-off
			if (MotorOut1<MOTORS_IDLE_VALUE) MotorOut1=MOTORS_IDLE_VALUE;
			if (MotorOut2<MOTORS_IDLE_VALUE) MotorOut2=MOTORS_IDLE_VALUE;
			if (MotorOut3<MOTORS_IDLE_VALUE) MotorOut3=MOTORS_IDLE_VALUE;
			if (MotorOut4<MOTORS_IDLE_VALUE) MotorOut4=MOTORS_IDLE_VALUE;
			
			
		} // End of ARMED & Throttle > Minimum
				
			
	}  // End of Throttle stick is NOT Down [Armed Could be True or not]
	

	Motor_GenerateOutputSignal();
	//LCD_SetPos(0,0);
	//itoa (MotorOut1,sXDeg,10);
	//LCD_WriteString(sXDeg);
	if (bResetTCNR1_X==true)
	{
		TCNT1_X_snapshot1= 0; // reset timeout
	}
	
	
}	

