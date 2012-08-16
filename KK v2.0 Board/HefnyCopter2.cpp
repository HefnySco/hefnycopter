/*
 * HefnyCopter2.cpp
 *
 * Created: 8/13/2012 6:30:08 PM
 *  Author: hefny
 */ 

#include <avr/io.h>
#include <math.h>
#include <avr/delay.h>
#include <avr/pgmspace.h>

#include "Include/GlobalValues.h"
#include "Include/HefnyCopter2.h"
#include "Include/IO_config.h"
#include "Include/LED.h"
#include "Include/Beeper.h"
#include "Include/Acc_Sensor.h"
#include "Include/ADC_PORT.h"
#include "Include/Gyro_Sensor.h"
#include "Include/LCD_ST7565.h"
#include "Include/Motor.h"
#include "Include/KeyBoard.h"

LED			mOrangeLED;
Beeper		mBeeper;
LCD_ST7565	mLCD;
Gyro_Sensor mGyroSensor;
Acc_Sensor  mAccSensor;
ADC_PORT    mAdcPort;
KeyBoard	mKeyBoard;
Motor		mMotor;

void Setup (void)
{
	
	RX_ROLL_DIR 		= INPUT;
	RX_PITCH_DIR 		= INPUT;
	RX_COLL_DIR   		= INPUT;
	RX_YAW_DIR   	 	= INPUT;
	RX_AUX_DIR   	 	= INPUT;

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
	
	// Timers
	TCCR1A = 0;	//Set timer 1 to run at 2.5MHz
	TCCR1B = 0;
	TCCR1C = 0;
	
	// Interrupts Settings
	EIMSK = 0b00000111;
	//        76543210
	PCICR = 0b00001010;
	//       76543210
	PCMSK3= 0b00000001;
	//       76543210
	PCMSK1= 0b00000001;

	mAccSensor.Init();
	mGyroSensor.Init();
	mAdcPort.Init();
	
	mKeyBoard.Init();
	mOrangeLED.FlashLED(200,4);
	//mBeeper.Beep(200,4);
	mKeyBoard.Init();
	mLCD.Init();
	_delay_ms(500);
   //mLCD.Write_Instruction(CMD_TEST);
   mLCD.Clear();
   mLCD.DrawString(0,0, (char*)"I am ready ABCDEFG abcde");
   mLCD.DrawLine(0,0,25,25,1);
   mLCD.DrawCircle(50,50,10,1);
   mLCD.Display();
}

int main(void)
{
	
	Setup();
	
    while(1)
    {
        Loop();
    }
}


void Loop(void)
{
	
	if ((mKeyBoard.KeyboardRead() & KEY_1))
	{
		LED_Orange = ON;
	}
	
}	


uint16_t TCNT1_X_snapshot=0;
int16_t cROLL;
int16_t cPITCH;
int16_t cYAW;
int16_t fROLL;
int16_t fPITCH;
int16_t fYAW;
bool bXQuadMode = false;	
bool bResetTCNR1_X = true;

void Loop2(void)
{
	
	if ((mKeyBoard.KeyboardRead() & KEY_1))
	{
		LED_Orange = OFF;
	}
	else
	{
		LED_Orange = ON;
	}
	bResetTCNR1_X = true;
	

	RxGetChannels();
	
	
	if (RxInCollective < STICKThrottle_ARMING) 
	{	// Throttle is LOW
		// Here you can add code without caring about delays. As there quad is already off and on land.
		// here we test different positions of sticks to enable arm/disarm, Quad/X-Quad
		
		//ReadGainValues(); // keep reading values of POTS here. as we can change the value while quad is armed. but sure it is on land and motors are off.
		// DisArm Check
		if ((bArmed == true) && (RxInYaw < STICK_LEFT))
		{
			bResetTCNR1_X  = false;
			if (TCNT1_X_snapshot==0)  TCNT1_X_snapshot = TCNT1_X; // start counting
			if ( (TCNT1_X- TCNT1_X_snapshot) > STICKPOSITION_MIN )
			{
				bArmed = false;
				LED_Orange = OFF;
				mOrangeLED.FlashLED (LED_LONG_TOGGLE,4);
				TCNT1_X_snapshot =0; // reset timer
			}
		}
		
		// Arm Check
		if ((bArmed == false) && (RxInYaw > STICK_RIGHT))
		{
			bResetTCNR1_X = false;
			if (TCNT1_X_snapshot==0)  TCNT1_X_snapshot = TCNT1_X; // start counting
			if ( (TCNT1_X- TCNT1_X_snapshot) > STICKPOSITION_MIN )
			{
				bArmed = true;
				LED_Orange = ON;
				mOrangeLED.FlashLED (LED_LONG_TOGGLE,4);
				//CalibrateGyros();
				//ReadGainValues();
				mOrangeLED.FlashLED (LED_SHORT_TOGGLE,4);
				TCNT1_X_snapshot =0; // reset timer
			}		
		}
		
		
		if (!bArmed)
		{	//set modes Quad , X-Quad
		
			if ((RxInRoll > STICK_RIGHT))
			{	// X-QUAD MODE
				bResetTCNR1_X = false;
				if (TCNT1_X_snapshot==0)  TCNT1_X_snapshot = TCNT1_X; // start counting
				if ( (TCNT1_X- TCNT1_X_snapshot) > STICKPOSITION_MIN )
				{
					bXQuadMode = true;
					LED_Orange = OFF;
					mOrangeLED.FlashLED (100,8);
					TCNT1_X_snapshot =0; // reset timer
				}		

			
			}
			else  if ((RxInRoll < STICK_LEFT))
			{	// QUAD COPTER MODE
				bResetTCNR1_X = false;
				if (TCNT1_X_snapshot==0)  TCNT1_X_snapshot = TCNT1_X; // start counting
				if ( (TCNT1_X- TCNT1_X_snapshot) > 16 )
				{
					bXQuadMode = false;
					LED_Orange = ON;
					mOrangeLED.FlashLED (LED_LONG_TOGGLE,4);
					TCNT1_X_snapshot =0; // reset timer
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
		
		if (!bArmed)
		{  // However we are still DisArmed
			MotorOut1 = 0;
			MotorOut2 = 0;
			MotorOut3 = 0;
			MotorOut4 = 0;
		}
		else
		{	// Armed & Throttle Stick > MIN . . . We should Fly now.
			
			
			/*
			*
			*	Calculate Balance Here
			*	
			*/
	
			
			
			/*
			*
			*	Pilot Control Logic.
			*	
			*/
	
			if (bXQuadMode==true)
			{
							
				MotorOut1 += RxInRoll ;
				MotorOut2 += RxInRoll ;
				MotorOut3 -= RxInRoll ;
				MotorOut4 -= RxInRoll ;
				
				MotorOut1 += RxInPitch;
				MotorOut2 -= RxInPitch;
				MotorOut3 += RxInPitch;
				MotorOut4 -= RxInPitch;
				
				MotorOut1 -= RxInYaw;
				MotorOut2 += RxInYaw;
				MotorOut3 += RxInYaw;
				MotorOut4 -= RxInYaw;
			}
			else
			{
				MotorOut2 += RxInRoll ;
				MotorOut3 -= RxInRoll ;
				
				MotorOut1 += RxInPitch ;
				MotorOut4 -= RxInPitch ;
		
				MotorOut1 -= RxInYaw ;
				MotorOut2 += RxInYaw ;
				MotorOut3 += RxInYaw ;
				MotorOut4 -= RxInYaw ;
			}
			
			
			// Save motors from turning-off
			if (MotorOut1<10) MotorOut1=10;
			if (MotorOut2<10) MotorOut2=10;
			if (MotorOut3<10) MotorOut3=10;
			if (MotorOut4<10) MotorOut4=10;
			
			
		} // End of ARMED & Throttle > Minimum
				
			
	}  // End of Throttle stick is NOT Down [Armed Could be True or not]
	

	mMotor.GenerateOutputSignal();
	
	if (bResetTCNR1_X==true)
	{
		TCNT1_X_snapshot= 0; // reset timeout
	}
		
				
}	


void RxGetChannels(void)
{
	
	
}