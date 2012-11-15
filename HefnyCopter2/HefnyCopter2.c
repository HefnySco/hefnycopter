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
#include "Include/UART.h"


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

	Initial_EEPROM_Config_Load();
	
	Config.QuadFlyingMode = QuadFlyingMode_PLUS;
	
	RX_Init();
	// Motors
	M1_DIR = OUTPUT;
	M2_DIR = OUTPUT;
	M3_DIR = OUTPUT;
	M4_DIR = OUTPUT;
	
	
	Buzzer_DIR = OUTPUT;
	LED_Orange_DIR = OUTPUT;
	
	
	// Sensors
	V_BAT  = INPUT;
	
	
	
	// Timers
	TCCR1A = 0;	//Set timer 1 to run at 2.5MHz
	TCCR1B = 0;
	TCCR1C = 0;
	
#ifdef PRIMARY_INPUT_RX
				// clear interrupts
#endif
//#ifdef UART_ENABLED
if (Config.RX_mode==RX_mode_UARTMode)
{
	UART_Init(10); //57600 = 20   115200=10
}	
	//15,11,10,9,8,7,6,5,4,3,2,1

//#endif


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
	// Stick Commands are only available for Secondary Receiver and when Stick is calibrated.
	UIEnableStickCommands=false;  
	Setup();
	
    SystemErrorType = SYS_ERR_NON;

	DataPtr = (uint8_t *) (&Sensors_Latest);
	DataCounter=0;
	while (Config.IsESCCalibration==ESCCalibration_ON)		
	{
		Beeper_Beep(BEEP_SHORT,2);
		
		Menu_LoadPage(PAGE_ESC_CALIBRATION);
		while (1)
		{  // loop forever
			LoopESCCalibration();
			Loop();
		}
	}			

	// Never go to MainLoop "fly loop" unless Sensors & RX is calibrated.
	// This loop to protect against any bug that might make the quad start or KB stick click
	// as in this case crash is a must.
	while (!(Config.IsCalibrated & CALIBRATED_SENSOR) || !(Config.IsCalibrated & CALIBRATED_Stick_SECONDARY))
	{
		Loop();
	}
	
	
	// Simulate
	//IsArmed = true;
	//Menu_LoadPage (PAGE_HOME_ARMED);
	
				
	while(1)
    {
		//LoopESCCalibration();
    	MainLoop();
		//SlowerLoop();
    }
}


void LoopESCCalibration (void)
{
	RX_CopyLatestReceiverValues();
	
	MotorOut[0] = RX_Latest[ActiveRXIndex][RXChannel_THR];
	MotorOut[1] = RX_Latest[ActiveRXIndex][RXChannel_THR];
	MotorOut[2] = RX_Latest[ActiveRXIndex][RXChannel_THR];
	MotorOut[3] = RX_Latest[ActiveRXIndex][RXChannel_THR];		

	Motor_GenerateOutputSignal();
	
}


/*
	We are in this loop because the system is not calibrated.
	This is the minimum functionality.
*/
void Loop(void)
{
	RX_CopyLatestReceiverValues();
	
	if (TCNT_X_snapshot2==0) TCNT_X_snapshot2 = TCNT2_X;
	if ( (TCNT2_X- TCNT_X_snapshot2) > LCD_RefreashRate )  
	{
		Menu_MenuShow();	
		TCNT_X_snapshot2=0;
	}		

}

//
//void SlowerLoop(void)
//{
	//
	//
//}
//
/*
	This is the main loop of the application.
*/
void MainLoop(void)
{
	
	RX_CopyLatestReceiverValues();
	RX_Snapshot[RXChannel_THR] = RX_Latest[ActiveRXIndex][RXChannel_THR];
	Sensors_ReadAll();	
	
	
	
	
	// simulate
	//RX_Latest[RXChannel_THR]=500;
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
      CurrentTCNT1_X = TCNT1_X;
    }

	IMU_P2D(); 
	bResetTCNR1_X = true;
	
	
	if (Config.RX_mode==RX_mode_BuddyMode)
	{   // in Buddy mode AUX channel is used for instance switching.
		if (IS_TX2_GOOD)
		{
			if (RX_Latest[1/*Always read from Secondary*/][RXChannel_AUX] < STICK_RIGHT)
			{
				ActiveRXIndex = 0;		// use Primary RX
			}
			else
			{
				ActiveRXIndex = 1;		// use Secondary RX
			}
		}			
	}			
	
		
	// Slow Actions inside
	// HINT: you can try to skip this if flying to save time for more useful tasks as user cannot access menu when flying
	if (TCNT_X_snapshot2==0) TCNT_X_snapshot2 = CurrentTCNT1_X;
	else if ( ((CurrentTCNT1_X- TCNT_X_snapshot2) > 4) )  // TCNT1_X ticks in 32.768us
	{
		Menu_MenuShow();
		
		if (Config.VoltageAlarm > 0)
		{
			//Sensor_GetBattery();
			if (Sensors_Latest[V_BAT_Index] < Config.VoltageAlarm)
			{
				
				SystemErrorType = SET_SYS_ERR_VOLTAGE;
			}
			else
			{
				SystemErrorType = CLR_SYS_ERR_VOLTAGE;
				
			}
		}	
		if (SystemErrorType != SYS_ERR_NON)
		{
			Buzzer =ON ;	
		}
		else
		{
			Buzzer = OFF;
		}
		
		
		
		if ((IsArmed == true) && (RX_Snapshot[RXChannel_THR] < STICKThrottle_ARMING+160))
		{ // calibrate when start flying
			DynamicCalibration();
		}			
		TCNT_X_snapshot2=0;
	}		
	

	
	if (RX_Snapshot[RXChannel_THR] < STICKThrottle_ARMING) 
	{	
		
		// Throttle is LOW
		// Here you can add code without caring about delays. As there quad is already off and on land.
		// here we test different positions of sticks to enable arm/disarm, Quad/X-Quad
		HandleSticksForArming();
		
		// Stop motors if Throttle Stick is less than minimum.
		ZEROMotors();
		
		PID_GyroTerms[0].I=0;
		PID_GyroTerms[1].I=0;
		PID_GyroTerms[2].I=0;
		PID_AccTerms [0].I=0;
		PID_AccTerms [1].I=0;
		PID_AccTerms [2].I=0;
		
		// Send Setting Data only when Throttle is down.
		if (Config.RX_mode==RX_mode_UARTMode)
		{
			Send_Data("C",1);
			Send_Data(&Config,72);
			Send_Data("E",1);
		}
	}
	else
	{	// Throttle stick is NOT Down
		
		if (IsArmed==false)
		{  // However we are still DisArmed
			ZEROMotors();
			
			// Sticks as Keyboard --- we are already disarmed to reach here.
			HandleSticksAsKeys();
			
		}
		else
		{	// MOTORS ARE ON HERE
			
			
			TCNT_X_snapshotAutoDisarm =0; // ZERO [user may disarm then fly slowly..in this case the qud will disarm once he turned off the stick...because the counter counts once the quad is armed..e.g. if it takes n sec to disarm automatically..user took n-1 sec keeping the stick low after arming then it will take 1 sec to disarm again after lowing the stick under STICKThrottle_ARMING
			
			// Armed & Throttle Stick > MIN . . . We should Fly now.
			
			RX_Snapshot[RXChannel_AIL] = (RX_Latest[ActiveRXIndex][RXChannel_AIL] * 3) / 5 ;
			RX_Snapshot[RXChannel_ELE] = (RX_Latest[ActiveRXIndex][RXChannel_ELE] * 3) / 5;
			RX_Snapshot[RXChannel_RUD] = (RX_Latest[ActiveRXIndex][RXChannel_RUD] * 3) / 5 ;
			
			int16_t Landing;
			if (RX_Latest[1/*Always read from Secondary*/][RXChannel_AUX] < STICK_RIGHT)
			{
				LED_Orange=OFF;
				Landing = IMU_HeightKeeping();
			}
			else
			{
				LED_Orange=ON;
				Landing =0;
			}
			
			MotorOut[0] = RX_Snapshot[RXChannel_THR] + Landing;
			MotorOut[1] = RX_Snapshot[RXChannel_THR] + Landing;
			MotorOut[2] = RX_Snapshot[RXChannel_THR] + Landing;
			MotorOut[3] = RX_Snapshot[RXChannel_THR] + Landing;		
			
			
	
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
				MotorOut[0] -= gyroPitch ;
				MotorOut[3] += gyroPitch ; 
				MotorOut[1] -= gyroRoll  ;
				MotorOut[2] += gyroRoll  ;
				MotorOut[0] += gyroYaw;
				MotorOut[3] += gyroYaw;
				MotorOut[1] -= gyroYaw;
				MotorOut[2] -= gyroYaw;
				
		
			
			/*
			*
			*	Pilot Control Logic.
			*	
			*/
	
			if (Config.QuadFlyingMode==QuadFlyingMode_X)
			{
							
				MotorOut[0] += RX_Snapshot[RXChannel_AIL] ;
				MotorOut[1] += RX_Snapshot[RXChannel_AIL] ;
				MotorOut[2] -= RX_Snapshot[RXChannel_AIL] ;
				MotorOut[3] -= RX_Snapshot[RXChannel_AIL] ;
				
				MotorOut[0] += RX_Snapshot[RXChannel_ELE];
				MotorOut[1] -= RX_Snapshot[RXChannel_ELE];
				MotorOut[2] += RX_Snapshot[RXChannel_ELE];
				MotorOut[3] -= RX_Snapshot[RXChannel_ELE];
				
				MotorOut[0] -= RX_Snapshot[RXChannel_RUD];
				MotorOut[1] += RX_Snapshot[RXChannel_RUD];
				MotorOut[2] += RX_Snapshot[RXChannel_RUD];
				MotorOut[3] -= RX_Snapshot[RXChannel_RUD];
			
			}
			else
			{
				
				MotorOut[1] += RX_Snapshot[RXChannel_AIL] ;
				MotorOut[2] -= RX_Snapshot[RXChannel_AIL] ;
				
				MotorOut[0] += RX_Snapshot[RXChannel_ELE] ;
				MotorOut[3] -= RX_Snapshot[RXChannel_ELE] ;
		
				MotorOut[0] -= RX_Snapshot[RXChannel_RUD] ;
				MotorOut[1] += RX_Snapshot[RXChannel_RUD] ;
				MotorOut[2] += RX_Snapshot[RXChannel_RUD] ;
				MotorOut[3] -= RX_Snapshot[RXChannel_RUD] ;
				
			}
			
			
			// Save motors from turning-off
            if (MotorOut[0]<MOTORS_IDLE_VALUE) MotorOut[0]=MOTORS_IDLE_VALUE;
            if (MotorOut[1]<MOTORS_IDLE_VALUE) MotorOut[1]=MOTORS_IDLE_VALUE;
            if (MotorOut[2]<MOTORS_IDLE_VALUE) MotorOut[2]=MOTORS_IDLE_VALUE;
            if (MotorOut[3]<MOTORS_IDLE_VALUE) MotorOut[3]=MOTORS_IDLE_VALUE;
			
		
			// Sending Sensors & Motor Data 
			if (Config.RX_mode==RX_mode_UARTMode)
			{
				//LED_Orange=~LED_Orange;
				Send_Data("S",1);
				Send_Data(Sensors_Latest,12);
				Send_Data(MotorOut,8);
				Send_Data("E",1);
			}
			
			
		} // End of ARMED & Throttle > Minimum
				
			
	}  // End of Throttle stick is NOT Down [Armed Could be True or not]
	
	if ((!IS_TX2_GOOD)) // if no signal and there is no AutoLandingMode.
	{
		ZEROMotors();
		if (IsArmed==true)
		{
			Motor_GenerateOutputSignal();	
			Disarm();	
		}
		SystemErrorType = SET_SYS_ERR_SIGNAL;
		
		//return ; // Do nothing all below depends on TX.
	}
	
	Motor_GenerateOutputSignal();	
	
	if (bResetTCNR1_X==true)
	{
		TCNT1_X_snapshot1= 0; // reset timeout
	}
	
	
}	





// This function is never called if there is a calibration issue.
void HandleSticksForArming (void)
{
	if ((UIEnableStickCommands==false) || (ActiveRXIndex!=1) || (!IS_TX2_GOOD)) return ; // you cannot use Primary to Arm and Disarm
	SystemErrorType = CLR_SYS_ERR_SIGNAL;
	
	if (TCNT1_X_snapshot1==0)  TCNT1_X_snapshot1 = CurrentTCNT1_X; // start counting
		
		/////ReadGainValues(); // keep reading values of POTS here. as we can change the value while quad is armed. but sure it is on land and motors are off.
		// DisArm Check
		if (IsArmed == true) 
		{
			if (RX_Latest[ActiveRXIndex][RXChannel_RUD] < STICK_RIGHT)
			{ // Check DisArming manually.
				bResetTCNR1_X  = false;
				if ( (CurrentTCNT1_X - TCNT1_X_snapshot1) > STICKPOSITION_LONG_TIME )
				{
					Disarm();
					return ;
				}
			}
			
			if (Config.AutoDisarm!=0)
			{ // check auto disArm
				if (TCNT_X_snapshotAutoDisarm==0) TCNT_X_snapshotAutoDisarm = CurrentTCNT1_X;
				if ((CurrentTCNT1_X - TCNT_X_snapshotAutoDisarm) > (DISARM_TIME * Config.AutoDisarm))
				{
					Disarm();
					return ;
				}
			}
			
		}			
		
		if (IsArmed == false) 
		{
			if (RX_Latest[ActiveRXIndex][RXChannel_RUD] > STICK_LEFT)
			{	// Armin Check
				bResetTCNR1_X = false;
				if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_LONG_TIME )
				{
					if ((Config.RX_mode==RX_mode_BuddyMode) && (!IS_TX1_GOOD)) return; // in Buddy mode you cannot arm is there is no signal from TX1
					//TODO: Calibrate Gyros Here to get used on Temperature
					Arm();
					return ;
				}
			}					
		
			//set modes Quad , X-Quad
		
			if (RX_Latest[ActiveRXIndex][RXChannel_AIL]  > STICK_LEFT)
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
			 if ((RX_Latest[ActiveRXIndex][RXChannel_AIL]  < STICK_RIGHT))
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


// This function is never called if there is a calibration issue.
void HandleSticksAsKeys (void)
{
		if ((UIEnableStickCommands==false) || (ActiveRXIndex!=1) || (!IS_TX2_GOOD))  return ; // you cannot use Primary as keys

	
		if (RX_Snapshot[RXChannel_THR] > STICKThrottle_HIGH)
		{ // if Throttle is high and stick are calibrated
		
				if (TCNT1_X_snapshot1==0)  TCNT1_X_snapshot1 = CurrentTCNT1_X; // start counting
				
	 			if ((RX_Latest[ActiveRXIndex][RXChannel_ELE]) > STICK_LEFT) 
				{
					bResetTCNR1_X = false;
					if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_SHORT_TIME )
					{
						_TXKeys = KEY_3;
						TCNT1_X_snapshot1 =0; // reset timer
					}		
				
				}
				else if ((RX_Latest[ActiveRXIndex][RXChannel_ELE]) < STICK_RIGHT) 
				{
					bResetTCNR1_X = false;
					if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_SHORT_TIME )
					{
						_TXKeys = KEY_2;
						TCNT1_X_snapshot1 =0; // reset timer
					}		
				
				}		 	 
				
				if ((RX_Latest[ActiveRXIndex][RXChannel_AIL]) > STICK_LEFT) 
				{
					bResetTCNR1_X = false;
					if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_SHORT_TIME )
					{
						_TXKeys = KEY_4;
						TCNT1_X_snapshot1 =0; // reset timer
					}		
				
				}
				else if ((RX_Latest[ActiveRXIndex][RXChannel_AIL]) < STICK_RIGHT) 
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

/*
// STOPS MOTORS IMMEDIATLY
*/
void ZEROMotors()
{
	MotorOut[0] = 0;
	MotorOut[1] = 0;
	MotorOut[2] = 0;
	MotorOut[3] = 0;
	Motor_GenerateOutputSignal();
}