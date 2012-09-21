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
#include "Include/IMU.h"
#include "Include/Math.h"
#include "Include/Arming.h"
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
	
	sei();
	
	delay_ms(20);
    
}


int main(void)
{
	
	Setup();
	Config.QuadFlyingMode = QuadFlyingMode_PLUS; 
			
				 //Pitch_Ratio = ((double)(Config.GyroParams[0].maxDest - Config.GyroParams[0].minDest)/(double)(Config.GyroParams[0].maxSource - Config.GyroParams[0].minSource));
			 //Yaw_Ratio = ((double)(Config.GyroParams[1].maxDest - Config.GyroParams[1].minDest)/(double)(Config.GyroParams[1].maxSource - Config.GyroParams[1].minSource));
			 //Acc_Ratio = ((double)(Config.AccParams.maxDest - Config.AccParams.minDest)/(double)(Config.AccParams.maxSource - Config.AccParams.minSource));
			
	while (Config.IsESCCalibration==ESCCalibration_ON)		
	{
		Beeper_Beep(100,2);
		
		Menu_LoadPage(PAGE_ESC_CALIBRATION);
		while (1)
		{  // loop forever
			LoopESCCalibration();
			Loop();
		}
	}			

	while ((!(Config.IsCalibrated & CALIBRATED_SENSOR)) || (!(Config.IsCalibrated & CALIBRATED_Stick)))
	{
		Loop();
	}
	
	
	// Simulate
	//IsArmed = true;
	//Menu_LoadPage (PAGE_HOME_ARMED);
	
				
	while(1)
    {
    	MainLoop();
    }
}


void LoopESCCalibration (void)
{
	RX_CopyLatestReceiverValues();
	
	MotorOut1 = RX_Latest[RXChannel_THR];
	MotorOut2 = RX_Latest[RXChannel_THR];
	MotorOut3 = RX_Latest[RXChannel_THR];
	MotorOut4 = RX_Latest[RXChannel_THR];		

	Motor_GenerateOutputSignal();
	
}


/*
	We are in this loop because the system is not calibrated.
	This is the minimum functionality.
*/
void Loop(void)
{
	
	if (TCNT_X_snapshot2==0) TCNT_X_snapshot2 = TCNT2_X;
	if ( (TCNT2_X- TCNT_X_snapshot2) > LCD_RefreashRate )  
	{
		Menu_MenuShow();	
		TCNT_X_snapshot2=0;
	}		

}


/*
	This is the main loop of the application.
*/
void MainLoop(void)
{
	
	RX_CopyLatestReceiverValues();
	// simulate
	//RX_Latest[RXChannel_THR]=500;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      CurrentTCNT1_X = TCNT1_X;
    }
	Sensors_ReadAll();
	IMU_P2D();
	bResetTCNR1_X = true;
	
  
	
	// HINT: you can try to skip this if flying to save time for more useful tasks as user cannot access menu when flying
		
	
		if (TCNT_X_snapshot2==0) TCNT_X_snapshot2 = CurrentTCNT1_X;
		else if ( ((CurrentTCNT1_X- TCNT_X_snapshot2) > 2) )  // TCNT1_X ticks in 32.768us
		{
			Menu_MenuShow();	
			TCNT_X_snapshot2=0;
		}		
	
	Config.AutoDisarm=1;
	
	if (RX_Good != TX_GOOD) 
	{
		return ; // Do nothing all below depends on TX.
	}	
	
	if (RX_Latest[RXChannel_THR] < STICKThrottle_ARMING) 
	{	
		
		// Throttle is LOW
		// Here you can add code without caring about delays. As there quad is already off and on land.
		// here we test different positions of sticks to enable arm/disarm, Quad/X-Quad
		HandleSticksForArming();
		
		// Stop motors if Throttle Stick is less than minimum.
		MotorOut1 = 0;
		MotorOut2 = 0;
		MotorOut3 = 0;
		MotorOut4 = 0;
		
		PID_Terms[0].I=0;
		PID_Terms[1].I=0;
		PID_Terms[2].I=0;
	}
	else
	{	// Throttle stick is NOT Down
		
		if (IsArmed==false)
		{  // However we are still DisArmed
			MotorOut1 = 0;
			MotorOut2 = 0;
			MotorOut3 = 0;
			MotorOut4 = 0;
			
			// Sticks as Keyboard --- we are already disarmed to reach here.
			HandleSticksAsKeys();
		}
		else
		{	
			
			TCNT_X_snapshotAutoDisarm =0; // ZERO [user may disarm then fly slowly..in this case the qud will disarm once he turned off the stick...because the counter counts once the quad is armed..e.g. if it takes n sec to disarm automatically..user took n-1 sec keeping the stick low after arming then it will take 1 sec to disarm again after lowing the stick under STICKThrottle_ARMING
			
			// Armed & Throttle Stick > MIN . . . We should Fly now.
			
			/*
			This code is good, however if you take-off from a non-flat ground then quad will try to bend similar to ground...as it took it as a calibration point.
			*/
			////if (RX_Latest[RXChannel_THR] <( STICKThrottle_ARMING + 80)) // calibrate again before leaving ground to average vibPitch_Rations.
			////{
				////Config.Sensor_zero[GYRO_X_Index] = (Config.Sensor_zero[GYRO_X_Index] + ADCPort_Get(GYRO_X_PNUM))/2;
				////Config.Sensor_zero[GYRO_Y_Index] = (Config.Sensor_zero[GYRO_Y_Index] + ADCPort_Get(GYRO_Y_PNUM))/2;
				////Config.Sensor_zero[GYRO_Z_Index] = (Config.Sensor_zero[GYRO_Z_Index] + ADCPort_Get(GYRO_Z_PNUM))/2;
				////Config.Sensor_zero[ACC_X_Index]  = (Config.Sensor_zero[ACC_X_Index] + ADCPort_Get(ACC_X_PNUM))/2;
				////Config.Sensor_zero[ACC_Y_Index]  = (Config.Sensor_zero[ACC_Y_Index] + ADCPort_Get(ACC_Y_PNUM))/2;
				////
			////}
		//////
				
			
			MotorOut1 = RX_Latest[RXChannel_THR];
			MotorOut2 = RX_Latest[RXChannel_THR];
			MotorOut3 = RX_Latest[RXChannel_THR];
			MotorOut4 = RX_Latest[RXChannel_THR];		
	
			
	
			/*
			*
			*	Stabilization Logic.
			*	The logic is independent of Quad configuPitch_Ration 
			*/
			
			//IMU_CalculateAngles ();
			//double tCompAngleY = CompAngleY * CompAngleY;
			//double tCompAngleX = CompAngleX * CompAngleY;
			
			//gyroPitch = ScaleSensor (tCompAngleY,&(Config.AccParams),Acc_Ratio);
			//gyroRoll =  ScaleSensor (tCompAngleX,&(Config.AccParams),Acc_Ratio);
			//gyroYaw   = ScaleSensor (Sensors_Latest[GYRO_Z_Index],&(Config.GyroParams[1]),Yaw_Ratio);
			
		
			/*
			*
			*	Self Leveling
			*/
				
				/*
				if (Config.SelfLevelMode == IMU_SelfLevelMode)
				{
					IMU_Kalman();
				}
				else
				{
					
					IMU_P2D();
				}
				*/
				//gyroPitch = gyroPitch * (-1);
				//gyroRoll  = gyroRoll  * (-1);
				MotorOut1 -= gyroPitch ;
				MotorOut4 += gyroPitch ; 
				MotorOut2 -= gyroRoll  ;
				MotorOut3 += gyroRoll  ;
				MotorOut1 += gyroYaw;
				MotorOut4 += gyroYaw;
				MotorOut2 -= gyroYaw;
				MotorOut3 -= gyroYaw;
				
		
			
			
			/*
			*
			*	Pilot Control Logic.
			*	
			*/
	
			
			RX_Latest[RXChannel_AIL] = (RX_Latest[RXChannel_AIL] * 3) / 5 ;
			RX_Latest[RXChannel_ELE] = (RX_Latest[RXChannel_ELE] * 3) / 5;
			RX_Latest[RXChannel_RUD] = (RX_Latest[RXChannel_RUD] * 3) / 5 ;
			
	
			if (Config.QuadFlyingMode==QuadFlyingMode_X)
			{
							
				MotorOut1 += RX_Latest[RXChannel_AIL] ;
				MotorOut2 += RX_Latest[RXChannel_AIL] ;
				MotorOut3 -= RX_Latest[RXChannel_AIL] ;
				MotorOut4 -= RX_Latest[RXChannel_AIL] ;
				
				MotorOut1 += RX_Latest[RXChannel_ELE];
				MotorOut2 -= RX_Latest[RXChannel_ELE];
				MotorOut3 += RX_Latest[RXChannel_ELE];
				MotorOut4 -= RX_Latest[RXChannel_ELE];
				
				MotorOut1 -= RX_Latest[RXChannel_RUD];
				MotorOut2 += RX_Latest[RXChannel_RUD];
				MotorOut3 += RX_Latest[RXChannel_RUD];
				MotorOut4 -= RX_Latest[RXChannel_RUD];
			}
			else
			{
				
				MotorOut2 += RX_Latest[RXChannel_AIL] ;
				MotorOut3 -= RX_Latest[RXChannel_AIL] ;
				
				MotorOut1 += RX_Latest[RXChannel_ELE] ;
				MotorOut4 -= RX_Latest[RXChannel_ELE] ;
		
				MotorOut1 -= RX_Latest[RXChannel_RUD] ;
				MotorOut2 += RX_Latest[RXChannel_RUD] ;
				MotorOut3 += RX_Latest[RXChannel_RUD] ;
				MotorOut4 -= RX_Latest[RXChannel_RUD] ;
			}
			
			
			// Save motors from turning-off
			if (MotorOut1<MOTORS_IDLE_VALUE) MotorOut1=MOTORS_IDLE_VALUE;
			if (MotorOut2<MOTORS_IDLE_VALUE) MotorOut2=MOTORS_IDLE_VALUE;
			if (MotorOut3<MOTORS_IDLE_VALUE) MotorOut3=MOTORS_IDLE_VALUE;
			if (MotorOut4<MOTORS_IDLE_VALUE) MotorOut4=MOTORS_IDLE_VALUE;
			
			
		} // End of ARMED & Throttle > Minimum
				
			
	}  // End of Throttle stick is NOT Down [Armed Could be True or not]
	

	Motor_GenerateOutputSignal();
	
	if (bResetTCNR1_X==true)
	{
		TCNT1_X_snapshot1= 0; // reset timeout
	}
	
	
}	






void HandleSticksForArming (void)
{
	if (TCNT1_X_snapshot1==0)  TCNT1_X_snapshot1 = CurrentTCNT1_X; // start counting
		
		/////ReadGainValues(); // keep reading values of POTS here. as we can change the value while quad is armed. but sure it is on land and motors are off.
		// DisArm Check
		if (IsArmed == true) 
		{
			if (RX_Latest[RXChannel_RUD] < STICK_RIGHT)
			{ // Check DisArming manually.
				bResetTCNR1_X  = false;
				if ( (CurrentTCNT1_X - TCNT1_X_snapshot1) > STICKPOSITION_LONG_TIME )
				{
					Disarm();
					return ;
				}
			}
			
			if (Config.AutoDisarm==1)
			{ // check auto disArm
				if (TCNT_X_snapshotAutoDisarm==0) TCNT_X_snapshotAutoDisarm = CurrentTCNT1_X;
				if ((CurrentTCNT1_X - TCNT_X_snapshotAutoDisarm) > DISARM_TIME)
				{
					Disarm();
					return ;
				}
			}
			
		}			
		
		if (IsArmed == false) 
		{
			if (RX_Latest[RXChannel_RUD] > STICK_LEFT)
			{	// Armin Check
				bResetTCNR1_X = false;
				if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_LONG_TIME )
				{
					Arm();
					return ;
				}
			}					
		
			//set modes Quad , X-Quad
		
			if (RX_Latest[RXChannel_AIL]  > STICK_LEFT)
			{// X-QUAD MODE
				bResetTCNR1_X = false;
				if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_LONG_TIME )
				{
					Config.QuadFlyingMode=QuadFlyingMode_X;
					LED_FlashOrangeLED (LED_LONG_TOGGLE,8);
					TCNT1_X_snapshot1 =0; // reset timer
				}
			}			
			else 
			{
			 if ((RX_Latest[RXChannel_AIL]  < STICK_RIGHT))
				{	// QUAD COPTER MODE
					bResetTCNR1_X = false;
					if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_LONG_TIME )
					{
						Config.QuadFlyingMode=QuadFlyingMode_PLUS;
						LED_FlashOrangeLED (LED_LONG_TOGGLE,4);
						TCNT1_X_snapshot1 =0; // reset timer
					}		
			
				} 
			}			
			
		}
	
}



void HandleSticksAsKeys (void)
{
	
			if ((Config.IsCalibrated & CALIBRATED_SENSOR) && (Config.IsCalibrated & CALIBRATED_Stick) && RX_Latest[RXChannel_THR] > STICKThrottle_HIGH)
			{ // if Throttle is high and stick are calibrated
		
				if (TCNT1_X_snapshot1==0)  TCNT1_X_snapshot1 = CurrentTCNT1_X; // start counting
				
	 			if ((RX_Latest[RXChannel_ELE]) > STICK_LEFT) 
				{
					bResetTCNR1_X = false;
					if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_SHORT_TIME )
					{
						_TXKeys = KEY_3;
						TCNT1_X_snapshot1 =0; // reset timer
					}		
				
				}
				else if ((RX_Latest[RXChannel_ELE]) < STICK_RIGHT) 
				{
					bResetTCNR1_X = false;
					if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_SHORT_TIME )
					{
						_TXKeys = KEY_2;
						TCNT1_X_snapshot1 =0; // reset timer
					}		
				
				}		 	 
				
				if ((RX_Latest[RXChannel_AIL]) > STICK_LEFT) 
				{
					bResetTCNR1_X = false;
					if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_SHORT_TIME )
					{
						_TXKeys = KEY_4;
						TCNT1_X_snapshot1 =0; // reset timer
					}		
				
				}
				else if ((RX_Latest[RXChannel_AIL]) < STICK_RIGHT) 
				{
					bResetTCNR1_X = false;
					if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_SHORT_TIME )
					{
						_TXKeys = KEY_1;
						TCNT1_X_snapshot1 =0; // reset timer
					}		
				
				}		 		
			}	
			
}