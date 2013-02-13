/*
 * IMU.c
 *
 * Created: 30-Aug-12 8:52:26 AM
 *  Author: M.Hefny
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>


#include "../Include/typedefs.h"
#include "../Include/GlobalValues.h"
#include "../Include/Math.h"
#include "../Include/IMU.h"
#include "../Include/PID.h"
//#include "../Include/TriGonometry.h"
#include "../Include/DCM.h"








//////////////////////////////////////////////////////////////////////////
// inspired by link: http://scolton.blogspot.com/2012/09/a-bit-more-kk20-modding.html
// Although I implement PID and super position in http://hefnycopter.net/index.php/developing-source-code/22-quadcopter-control-function-layers.html
void IMU (void)
{
	
		double Alpha;	
		double Beta;
	
		Alpha = Config.GyroParams[0].ComplementaryFilterAlpha / 1000.0;
		Beta = 1- Alpha;
		CompGyroPitch = (double) (Alpha * CompGyroPitch) + (double) (Beta * Sensors_Latest[GYRO_PITCH_Index]);
		CompGyroRoll  = (double) (Alpha * CompGyroRoll)  + (double) (Beta * Sensors_Latest[GYRO_ROLL_Index]);
			
		if (nFlyingModes == FLYINGMODE_ACRO)
		{
			gyroPitch =	PID_Calculate (Config.GyroParams[0], &PID_GyroTerms[0],CompGyroPitch);	
			gyroRoll  = PID_Calculate (Config.GyroParams[0], &PID_GyroTerms[1],CompGyroRoll); 
		}
		else
		{	
			
			// Read ACC and Trims 
			double APitch = - Sensors_Latest[ACC_PITCH_Index] - Config.Acc_Pitch_Trim;
			double ARoll  = - Sensors_Latest[ACC_ROLL_Index]  - Config.Acc_Roll_Trim;
			
			// Calculate Angle using Gyro
			AnglePitch = (AnglePitch + (double)Sensors_Latest[GYRO_PITCH_Index] * 0.042) ;
			AngleRoll =  (AngleRoll  + (double)Sensors_Latest[GYRO_ROLL_Index]  * 0.042) ;
			Alpha = Config.AccParams[0].ComplementaryFilterAlpha / 1000.0; // TODO: optimize
			Beta = 1- Alpha;
			#define ACC_SMALL_ANGLE	40
			// if small angle then correct using ACC
			if ((APitch < ACC_SMALL_ANGLE) && (APitch > -ACC_SMALL_ANGLE)) 
			{
				AnglePitch = Alpha * AnglePitch + Beta * APitch;
			}
		
			if ((ARoll  < ACC_SMALL_ANGLE) && (ARoll  > -ACC_SMALL_ANGLE))
			{
				AngleRoll =  Alpha * AngleRoll + Beta * ARoll;
			
			}
		
			
			
			NavY = AnglePitch;
			NavX = AngleRoll;
			
			Alpha = Config.AccParams[1].ComplementaryFilterAlpha / 1000.0;
			Beta = 1- Alpha;
			CompAccZ = (double) (Alpha * CompAccZ) + (double) (Beta * (double)Sensors_Latest[ACC_Z_Index]);
			
			
			
			if ((Config.BoardOrientationMode==QuadFlyingMode_PLUS) && (Config.QuadFlyingMode==QuadFlyingMode_X))
			{
				NavY += ( -  (double)((float)RX_Snapshot[RXChannel_AIL]  / 2.5f));
				NavY += ( -  (double)((float)RX_Snapshot[RXChannel_ELE]  / 2.5f));	
				NavX += ( -  (double)((float)RX_Snapshot[RXChannel_AIL]  / 2.5f));
				NavX += ( +  (double)((float)RX_Snapshot[RXChannel_ELE]  / 2.5f));	
			}
			else if ((Config.BoardOrientationMode==QuadFlyingMode_PLUS) && (Config.QuadFlyingMode==QuadFlyingMode_PLUS))
			{
				NavY += ( - (double)((float)RX_Snapshot[RXChannel_ELE] / 2.5f));	
				NavX += ( - (double)((float)RX_Snapshot[RXChannel_AIL] / 2.5f));
			}					
			else if ((Config.BoardOrientationMode==QuadFlyingMode_X) && (Config.QuadFlyingMode==QuadFlyingMode_X))
			{
				NavY += ( - (double)((float)RX_Snapshot[RXChannel_ELE] / 2.5f));	
				NavX += ( - (double)((float)RX_Snapshot[RXChannel_AIL] / 2.5f));
			}
			else if ((Config.BoardOrientationMode==QuadFlyingMode_X) && (Config.QuadFlyingMode==QuadFlyingMode_PLUS))
			{
				NavY += ( +  (double)((float)RX_Snapshot[RXChannel_AIL]  / 2.5f));
				NavY += ( -  (double)((float)RX_Snapshot[RXChannel_ELE]  / 2.5f));	
				NavX += ( -  (double)((float)RX_Snapshot[RXChannel_AIL]  / 2.5f));
				NavX += ( -  (double)((float)RX_Snapshot[RXChannel_ELE]  / 2.5f));	
			}
				
			double Error;
			
			gyroPitch =	PID_Calculate_ACC (Config.AccParams[0], &PID_AccTerms[0],NavY); //AnglePitch);	
			gyroRoll  = PID_Calculate_ACC (Config.AccParams[0], &PID_AccTerms[1],NavX); //AngleRoll); 
		 
			gyroPitch += PID_Calculate (Config.GyroParams[0], &PID_GyroTerms[0],CompGyroPitch);	
			gyroRoll  += PID_Calculate (Config.GyroParams[0], &PID_GyroTerms[1],CompGyroRoll); 
		
		}
		
		// calculate YAW
		Alpha = Config.GyroParams[1].ComplementaryFilterAlpha / 1000.0;
		Beta = 1- Alpha;
		CompGyroZ = (double) (Alpha * CompGyroZ) + (double) (Beta * Sensors_Latest[GYRO_Z_Index]);
		gyroYaw = PID_Calculate (Config.GyroParams[1], &PID_GyroTerms[2],CompGyroZ -((double)((float)RX_Snapshot[RXChannel_RUD]  / 2.0f))); 
	
}


	
double IMU_HeightKeeping ()
{
	/*
	int16_t Landing = (100 - CompAccZ) ;
	Landing *= Config.AccParams[1]._P; // PID_Terms[2].I not used for YAW 
	Limiter(Landing , Config.AccParams[1]._PLimit);
	*/
	double Landing =0.0f;
	double Temp;
	if ((Config.RX_mode==RX_mode_UARTMode) && (IS_MISC_SENSOR_SONAR_ENABLED==true))
	{
		RX_SONAR_TRIGGER = HIGH;
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{	
			Temp = RX_SONAR_RAW; 
		}
		if (Temp < 500)
		{
			if (((Temp - LastAltitudeHold) > SONAR_ALTITUDE_HOLD_REGION) || ((Temp - LastAltitudeHold) < -SONAR_ALTITUDE_HOLD_REGION))
			{
				LastAltitudeHold = Temp;
			}
			AltDiff = LastAltitudeHold - Temp;	
			//AltDiff = PID_AccTerms[2].D2  - AltDiff ;

		}
		RX_SONAR_TRIGGER = LOW;
	}	
	//Landing = AltDiff * PID_AccTerms[2].P;
	Landing = PID_Calculate_ACC (Config.AccParams[1], &PID_AccTerms[2],-CompAccZ + AltDiff);
	
	return Landing;
	
}