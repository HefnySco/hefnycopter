/*
 * HefnyCopter2.cpp
 *
 * Created: 8/13/2012 6:30:08 PM
 *  Author: hefny
 */ 

#define F_CPU	20000000UL	// 20 MHz

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
#include "Include/PID.h"
#include "Include/Menu_Screen.h"

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
		  
		  
TRI
		
		M1					M2
	          \        / 
               \ ---  /
				  |
				  |
				  |
				  M3 / M4 (servo)	

*/


static uint8_t FlyingModesToggle;

void Setup (void)
{
	
	Menu_EnableAllItems();
	
	Initial_EEPROM_Config_Load();
	
	//Config.QuadFlyingMode = QuadFlyingMode_PLUS;
	
	RX_Init();
	if ((Config.RX_mode==RX_mode_UARTMode) && (IS_MISC_SENSOR_SONAR_ENABLED==true))
	{
		Ultrasonic_Init();
	}		
	// Motors
	M1_DIR = OUTPUT;
	M2_DIR = OUTPUT;
	M3_DIR = OUTPUT;
	M4_DIR = OUTPUT;
	M1 = 0;
	M2 = 0;
	M3 = 0;
	M4 = 0;
	
	Buzzer_DIR = OUTPUT;
	LED_Orange_DIR = OUTPUT;
	
	
	// Sensors
	V_BAT  = INPUT;
	
	
	
	// Timers
	TCCR1A = 0;	//Set timer 1 to run at 2.5MHz
	TCCR1B = 0;
	TCCR1C = 0;
	

if (Config.RX_mode==RX_mode_UARTMode)
{
	UART_Init(SERIAL_BAUD_RATE); //57600 = 20   115200=10
}	



	ADCPort_Init();
	Sensors_Init();
	KeyBoard_Init();
	Timer_Init();
	
	LCD_Init();
	LCD_Clear();
	
	Menu_MenuInit();
	
		
	
	
	sei();
	
	//delay_ms(30);
    
}


int main(void)
{
	// Stick Commands are only available for Secondary Receiver and when Stick is calibrated.
	
	UIEnableStickCommands=false;  
	Setup();
	SystemActions = SYS_ACT_NON;
    SystemErrorType = SYS_ERR_NON;
	nFlyingModes = FLYINGMODE_ACRO;
	FlyingModesToggle = HIGH;
	
	DataPtr = (uint8_t *) (&Sensors_Latest);
	DataCounter=0;
	
	// Never go to MainLoop "fly loop" unless Sensors & RX is calibrated.
	// This loop to protect against any bug that might make the quad start or KB stick click
	// as in this case crash is a must.
	LoopCalibration ();

	
	
	// This loop better be under the sensor/stick loop to avoid entering this mode is sticks are not calibrated.
	// This is no longer a condition after adding menuEnabled[PAGE_ESC_CALIBRATION]=0
	if (Config.IsESCCalibration==ESCCalibration_ON)		
	{
		LoopESCCalibration ();
		
	}			 

/*
	Mixer[Mixer_Quad_PLUS].Pitch[4] = {-1.0,0,1.0,0};
	Mixer[Mixer_Quad_PLUS].Roll[4]  = {0,-1.0,0,1.0};
//#define Mixer_Quad_X		1
//#define Mixer_TRI			2
*/

	Menu_EnableAllItems();
	
				
	while(1)
    {
		//LoopESCCalibration();
    	MainLoop();
		//SlowerLoop();
    }
}

// Never go to MainLoop "fly loop" unless Sensors & RX is calibrated.
// This loop to protect against any bug that might make the quad start or KB stick click
// as in this case crash is a must.
void LoopCalibration (void)
{
	menuEnabled[PAGE_STABILIZATION] =0;
	menuEnabled[PAGE_SELF_LEVELING] =0;
	menuEnabled[PAGE_ALT_HOLD]		=0;
	menuEnabled[PAGE_MISC_SETTING]  =0;
	menuEnabled[PAGE_ESC_CALIBRATION]=0; // u cannot make ESC Calibration as sticks are not ready for testing.
		
	while (!(Config.IsCalibrated & CALIBRATED_SENSOR) || !(Config.IsCalibrated & CALIBRATED_Stick_SECONDARY))
	{	
		Loop();
	}
}

void LoopESCCalibration (void)
{
	
	Menu_LoadPage(PAGE_HOME_ESC_CALIBRATION);
	while (1)
	{
		Loop();
	}		
		
}


/*
	We are in this loop because the system is not calibrated.
	This is the minimum functionality.
*/
void Loop(void)
{
	RX_CopyLatestReceiverValues(); // update RX_Latest also IS_TXn_GOOD
	
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

static uint16_t LowpassOutYaw;
void MainLoop(void)
{
	
	RX_CopyLatestReceiverValues();
	//RX_Snapshot_1 [RXChannel_THR]= RX_Snapshot[RXChannel_THR];
	RX_Snapshot   [RXChannel_THR]= RX_Latest[ActiveRXIndex][RXChannel_THR];
	Sensors_ReadAll();	
	
	
	
	
	ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
      CurrentTCNT1_X = TCNT1_X;
    }

	IMU();
 
	bResetTCNR1_X = true;
	
	
	if (Config.RX_mode==RX_mode_BuddyMode)
	{   // in Buddy mode AUX channel is used for instance switching.
		if (IS_TX2_GOOD)
		{
			if (RX_Latest[RX_MAIN][RXChannel_AUX] < STICK_RIGHT)
			{
				ActiveRXIndex = 0;		// use Primary RX
			}
			else
			{
				ActiveRXIndex = 1;		// use Secondary RX
			}
		}			
	}			
	
		
	////////// Slow Actions inside
	// HINT: you can try to skip this if flying to save time for more useful tasks as user cannot access menu when flying
	
	if (TCNT_X_snapshot2 == 0) TCNT_X_snapshot2 = CurrentTCNT1_X;
	else if ( ((CurrentTCNT1_X - TCNT_X_snapshot2) > 4) )  // TCNT1_X ticks in 32.768us
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
			Buzzer =~Buzzer ;	
		}
		else
		{
			Buzzer = OFF;
		}
		
		if (Config.RX_mode==RX_mode_UARTMode)
		{
				if ((FlyingModesToggle != LOW) && ( RX_Latest[RX_MAIN][RXChannel_AUX] < STICK_RIGHT ))
				{
					nFlyingModes = FLYINGMODE_ALTHOLD;
					FlyingModesToggle = LOW;
				} else
				if ((FlyingModesToggle != HIGH) && (RX_Latest[RX_MAIN][RXChannel_AUX] > STICK_LEFT ))
				{
					//LED_Orange=ON;
					nFlyingModes = FLYINGMODE_ACRO;
					FlyingModesToggle = HIGH;
				} else
				if ((FlyingModesToggle != MID) && (RX_Latest[RX_MAIN][RXChannel_AUX]< STICK_LEFT) && ( RX_Latest[RX_MAIN][RXChannel_AUX] > STICK_RIGHT ))
				{
					nFlyingModes = FLYINGMODE_LEVEL;
					FlyingModesToggle = MID;
					
				}
				
		}		
		//if ((IsArmed == true) && (RX_Snapshot[RXChannel_THR] < STICKThrottle_ARMING+160))
		//{ // calibrate when start flying
			//DynamicCalibration();
		//}			
		TCNT_X_snapshot2=0;
	}		
	
	//////////////// EOF Slow Loop
	
	if (RX_Snapshot[RXChannel_THR] < STICKThrottle_ARMING) 
	{	
		
		// Throttle is LOW
		// Here you can add code without caring about delays. As there quad is already off and on land.
		// here we test different positions of sticks to enable arm/disarm, Quad/X-Quad
		HandleSticksForArming();
		
		// Stop motors if Throttle Stick is less than minimum.
		ZEROMotors();
		ZERO_Is();
		IMU_Reset(); // reset angles for gyro [STABLE MODE]
	}
	else
	{	// Throttle stick is NOT Down .... TAKE CARE
		
		if (IsArmed==false)
		{  // However we are still DisArmed
			ZEROMotors();
			
			// Sticks as Keyboard --- we are already disarmed to reach here.
			HandleSticksAsKeys();
			
		}
		else
		{	// MOTORS ARE ON HERE .... DANGEROUS
			
			
			TCNT_X_snapshotAutoDisarm = 0; // ZERO [user may disarm then fly slowly..in this case the qude will disarm once he turned off the stick...because the counter counts once the quad is armed..e.g. if it takes n sec to disarm automatically..user took n-1 sec keeping the stick low after arming then it will take 1 sec to disarm again after lowing the stick under STICKThrottle_ARMING
			
			// Armed & Throttle Stick > MIN . . . We should Fly now.
			//RX_Snapshot_1 [RXChannel_AIL]= RX_Snapshot[RXChannel_AIL];
			//RX_Snapshot_1 [RXChannel_ELE]= RX_Snapshot[RXChannel_ELE];
			//RX_Snapshot_1 [RXChannel_RUD]= RX_Snapshot[RXChannel_RUD];
			RX_Snapshot	  [RXChannel_AIL] = (RX_Latest[ActiveRXIndex][RXChannel_AIL] * Config.StickScaling * 0.05 );
			RX_Snapshot   [RXChannel_ELE] = (RX_Latest[ActiveRXIndex][RXChannel_ELE] * Config.StickScaling * 0.05 ); 
			RX_Snapshot   [RXChannel_RUD] = (RX_Latest[ActiveRXIndex][RXChannel_RUD] * Config.StickScaling * 0.05 ); // version 0.9.9 
			
			
		
			
			// Add Throttle to Motors
			MotorOut[0] = RX_Snapshot[RXChannel_THR];
			MotorOut[1] = RX_Snapshot[RXChannel_THR];
			if (Config.FrameType == FRAMETYPE_QUADCOPTER)
			{
				MotorOut[2] = RX_Snapshot[RXChannel_THR];
				MotorOut[3] = RX_Snapshot[RXChannel_THR];	
			}
			else
			{
				MotorOut[2] = RX_Snapshot[RXChannel_THR];
			}

			
			// Add IMU control.
			if (Config.FrameType == FRAMETYPE_QUADCOPTER)
			{	// Quadcopter
				/*
				* The logic below depends on board orientation i.e. sensor orientation compared to motor directions.
				* the IMU in ACHRO mode is totally independent from user sticks ... so it does not matter how the user
				* flies his quad i.e. in X or PLUS .... it does not matter because it is handled in another code lines not here.
				* here we handle board orientation only.
				*/
				if (Config.BoardOrientationMode==QuadFlyingMode_X)
				{	// Board Orientation in X-Mode
					// {-1,1,1,-1} QUAD_ROL_X
					MotorOut[0] -= gyroRoll ;
					MotorOut[3] -= gyroRoll ;
					MotorOut[1] += gyroRoll ;
					MotorOut[2] += gyroRoll ;
					
					// {-1,-1,1,1} QUAD_AIL_X
					MotorOut[0] -= gyroPitch;
					MotorOut[1] -= gyroPitch;
					MotorOut[2] += gyroPitch;
					MotorOut[3] += gyroPitch;
					
				}
				else
				{	// Board Orientation in Plus-Mode
					// {0,1,0,-1} QUAD_ROL_PLUS
					MotorOut[1] += gyroRoll  ;
					MotorOut[3] -= gyroRoll  ;
				
					// {-1,0,1,0} QUAD_AIL_PLUS
					MotorOut[0] -= gyroPitch ;
					MotorOut[2] += gyroPitch ; 
				
				}
			}
			else
			{	// Balance Tri-Copter
				// NOT VALID if Board Orientation is X
					
				MotorOut[2] += gyroPitch; // * 1.0;
					
				gyroPitch    = gyroPitch * 0.5; // distribute pitch on two front motors .... half the effect. 
				MotorOut[0] -= gyroPitch  ;
				MotorOut[1] -= gyroPitch  ;
					
				gyroRoll = gyroRoll; // * 0.85;
				MotorOut[0] -= gyroRoll  ;
				MotorOut[1] += gyroRoll  ;
			}				
			
			
			/*
			*
			*	Pilot Control Logic.
			*	Handles signals from remote control in ACRO mode.
			*	in stabilization mode controls are added in IMU logic as angles.
			*/
			if (nFlyingModes == FLYINGMODE_ACRO)
			{
				if (Config.FrameType == FRAMETYPE_QUADCOPTER)
				{
				
					if (Config.QuadFlyingMode==QuadFlyingMode_X)
					{
						RX_Snapshot[RXChannel_AIL] = RX_Snapshot[RXChannel_AIL] * 0.63;		// because we fly X
						RX_Snapshot[RXChannel_ELE] = RX_Snapshot[RXChannel_ELE] * 0.63;
					
						// {0.63,-0.63,-0.63,0.63} QUAD_AIL_X
						MotorOut[0] += RX_Snapshot[RXChannel_AIL] ;
						MotorOut[3] += RX_Snapshot[RXChannel_AIL] ;
						MotorOut[1] -= RX_Snapshot[RXChannel_AIL] ;
						MotorOut[2] -= RX_Snapshot[RXChannel_AIL] ;
				
						// {0.63,0.63,-0.63,-0.63} QUAD_ELE_X
						MotorOut[0] += RX_Snapshot[RXChannel_ELE];
						MotorOut[1] += RX_Snapshot[RXChannel_ELE];
						MotorOut[2] -= RX_Snapshot[RXChannel_ELE];
						MotorOut[3] -= RX_Snapshot[RXChannel_ELE];
					
					}
					else
					{
				
						// {0.9,0,-0.9,0} QUAD_ELE_PLUS
						MotorOut[0] += RX_Snapshot[RXChannel_ELE] ; 
						MotorOut[2] -= RX_Snapshot[RXChannel_ELE] ; 
						// {0,-0.9,0,0.9} QUAD_AIL_PLUS	
						MotorOut[1] -= RX_Snapshot[RXChannel_AIL] ; 
						MotorOut[3] += RX_Snapshot[RXChannel_AIL] ;  
								
					}
				}
				else
				{ // TRICopter
					
						int8_t inv=1; // Flying in Y mode
						if (Config.QuadFlyingMode==QuadFlyingMode_X)
						{ // Flying in A mode
							inv = -1;
						}							
							// {0.5,0.5,1.1,X} TRI_ELE_FRONT
							MotorOut[2] -= inv * (RX_Snapshot[RXChannel_ELE]); 
							RX_Snapshot[RXChannel_ELE] = inv * RX_Snapshot[RXChannel_ELE] * 0.5;
							MotorOut[0] += RX_Snapshot[RXChannel_ELE] ; 
							MotorOut[1] += RX_Snapshot[RXChannel_ELE] ; 
						
							// {1,-1,0,X} TRI_AIL_FRONT
							RX_Snapshot[RXChannel_AIL] = inv * RX_Snapshot[RXChannel_AIL];
							MotorOut[0] += RX_Snapshot[RXChannel_AIL] ; 
							MotorOut[1] -= RX_Snapshot[RXChannel_AIL] ;  
				}					
			}
			
			// ACC-Z & SONAR
			if (nFlyingModes != FLYINGMODE_ACRO)
			{
				// in stabilization mode ... activate Acc-Z & Sonar if enabled.
				
				double Landing;
				
				Landing = IMU_HeightKeeping();
				MotorOut[0] += Landing;
				MotorOut[1] += Landing;
				if (Config.FrameType == FRAMETYPE_QUADCOPTER)
				{
					
					MotorOut[2] += Landing;
					MotorOut[3] += Landing;		
				}
				else
				{
					MotorOut[2] += Landing;
				}
													
										
			}
			
									
			if (Config.FrameType == FRAMETYPE_QUADCOPTER)
			{
				MotorOut[0] -= gyroYaw;
				MotorOut[2] -= gyroYaw;
				MotorOut[1] += gyroYaw;
				MotorOut[3] += gyroYaw;
			}
			else
			{
			
				MotorOut[3]  = (Config.ReverseYAW * gyroYaw) + SERVO_IN_MIDDLE; 
				MotorOut[3]  = MotorOut[3] - (Config.ReverseYAW * RX_Snapshot[RXChannel_RUD]);
			}						
			
			// Save motors from turning-off
            if (MotorOut[0]<MOTORS_IDLE_VALUE) MotorOut[0]=MOTORS_IDLE_VALUE;
            if (MotorOut[1]<MOTORS_IDLE_VALUE) MotorOut[1]=MOTORS_IDLE_VALUE;
            if (MotorOut[2]<MOTORS_IDLE_VALUE) MotorOut[2]=MOTORS_IDLE_VALUE;
            
									
			if (Config.FrameType == FRAMETYPE_QUADCOPTER)
			{
				if (MotorOut[3]<MOTORS_IDLE_VALUE) MotorOut[3]=MOTORS_IDLE_VALUE;
			}
			
		
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
			SystemErrorType = SET_SYS_ERR_SIGNAL; // only error if signal lost while arming
		}
		
		
		//return ; // Do nothing all below depends on TX.
	}
	
	Motor_GenerateOutputSignal();	
	
	if (bResetTCNR1_X==true)
	{
		TCNT1_X_snapshot1= 0; // reset timeout
	}
	
	
}	





// This function is never called if there is a calibration issue.
// called in true section of if (RX_Snapshot[RXChannel_THR] < STICKThrottle_ARMING) 
void HandleSticksForArming (void)
{
	
	if ((UIEnableStickCommands==false) || (ActiveRXIndex!=1) || (!IS_TX2_GOOD)) return ; // you cannot use Primary to Arm and Disarm
	SystemErrorType = CLR_SYS_ERR_SIGNAL;
	
	if (TCNT1_X_snapshot1==0)  TCNT1_X_snapshot1 = CurrentTCNT1_X; // start counting
		
		// DisArm Check
		if (IsArmed == true) 
		{
			if (RX_Latest[ActiveRXIndex][RXChannel_RUD] > STICK_LEFT)
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
				if (TCNT_X_snapshotAutoDisarm > CurrentTCNT1_X ) TCNT_X_snapshotAutoDisarm = 0; // the CurrentTCNT1_X was high so the disarm condition never true.
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
			//int16_t Stick = (Config.RX_Mid[ActiveRXIndex][RXChannel_RUD] + RX_Latest[ActiveRXIndex][RXChannel_RUD] * RX_Div_Factor);
			if (RX_Latest[ActiveRXIndex][RXChannel_RUD]  < STICK_RIGHT)
			{	// Armin Check
				bResetTCNR1_X = false;
				if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_LONG_TIME )
				{
					if ((Config.RX_mode==RX_mode_BuddyMode) && (!IS_TX1_GOOD)) return; 
					// in Buddy mode you cannot arm is there is no signal from TX1
					
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
			else if ((RX_Latest[ActiveRXIndex][RXChannel_AIL]  < STICK_RIGHT))
				{	// QUAD COPTER MODE
					bResetTCNR1_X = false;
					if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_LONG_TIME )
					{
						Config.QuadFlyingMode=QuadFlyingMode_PLUS;
						LED_FlashOrangeLED (LED_LONG_TOGGLE,4);
						TCNT1_X_snapshot1 =0; // reset timer
					}		
			
				} 
				
			// set mode ACRO , Leveling
			else if ((RX_Latest[ActiveRXIndex][RXChannel_ELE]  < STICK_RIGHT))
				{	//nFlyingModes = FLYINGMODE_LEVEL;
					bResetTCNR1_X = false;
					if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_LONG_TIME )
					{
						nFlyingModes = FLYINGMODE_LEVEL;
						LED_FlashOrangeLED (LED_LONG_TOGGLE,8);
						TCNT1_X_snapshot1 =0; // reset timer
					}		
			
				} 
			else if ((RX_Latest[ActiveRXIndex][RXChannel_ELE]  > STICK_LEFT))
				{	//nFlyingModes = FLYINGMODE_ACRO;
					bResetTCNR1_X = false;
					if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_LONG_TIME )
					{
						nFlyingModes = FLYINGMODE_ACRO;
						LED_FlashOrangeLED (LED_LONG_TOGGLE,4);
						TCNT1_X_snapshot1 =0; // reset timer
					}		
			
				} 
		}
	
}


// This function is never called if there is a calibration issue.
// called in FALSE section of if (RX_Snapshot[RXChannel_THR] < STICKThrottle_ARMING) 
void HandleSticksAsKeys (void)
{
		if ((UIEnableStickCommands==false) || (!IS_TX2_GOOD))  return ; // you cannot use Primary as keys

		// if Throttle is high and stick are calibrated
		
		if (TCNT1_X_snapshot1==0)  TCNT1_X_snapshot1 = CurrentTCNT1_X; // start counting
				
	 	if ((RX_Latest[RX_MAIN][RXChannel_ELE]) > STICK_LEFT) 
		{
			bResetTCNR1_X = false;
			if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_SHORT_TIME )
			{
				_TXKeys = KEY_3;
				TCNT1_X_snapshot1 =0; // reset timer
			}		
		
		}
		else if ((RX_Latest[RX_MAIN][RXChannel_ELE]) < STICK_RIGHT) 
		{
			bResetTCNR1_X = false;
			if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_SHORT_TIME )
			{
				_TXKeys = KEY_2;
				TCNT1_X_snapshot1 =0; // reset timer
			}		
		
		}		 	 
		
		if ((RX_Latest[RX_MAIN][RXChannel_AIL]) > STICK_LEFT) 
		{
			bResetTCNR1_X = false;
			if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_SHORT_TIME )
			{
				_TXKeys = KEY_4;
				TCNT1_X_snapshot1 =0; // reset timer
			}		
		
		}
		else if ((RX_Latest[RX_MAIN][RXChannel_AIL]) < STICK_RIGHT) 
		{
			bResetTCNR1_X = false;
			if ( (CurrentTCNT1_X- TCNT1_X_snapshot1) > STICKPOSITION_SHORT_TIME )
			{
				_TXKeys = KEY_1;
				TCNT1_X_snapshot1 =0; // reset timer
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
	if (Config.FrameType == FRAMETYPE_TRICOPTER)
	{
		MotorOut[3] = SERVO_IN_MIDDLE;
	}	
	else
	{
		MotorOut[3] = 0;
	}		
		
	Motor_GenerateOutputSignal();
}