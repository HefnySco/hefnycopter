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
//#include "../Include/DCM.h"


// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void RotateV() 
{
	///*
	//// inspired by to Multiwii
	//void rotateV(struct fp_vector *v,float* delta) {
	  //fp_vector v_tmp = *v;
	  //v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
	  //v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
	  //v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X; 
	//*/
	//double oAnglePitch = AnglePitch;
	//double oAngleRoll = AngleRoll;
	//double oAngleZ     = AngleZ     ;
	AngleZ     -= (CompGyroRoll  * AngleRoll )  * GYRO_RATE + ( CompGyroPitch * AnglePitch) * GYRO_RATE;
	AngleRoll  += (CompGyroRoll  * AngleZ )     * GYRO_RATE - ( CompGyroZ     * AnglePitch) * GYRO_RATE;
	AnglePitch += (CompGyroPitch * AngleZ )		* GYRO_RATE + ( CompGyroZ     * AngleRoll ) * GYRO_RATE;

}





void IMU_Reset()
{
	
	AnglePitch=0;
	AngleRoll=0;
	AngleZ=0;
	
}
//////////////////////////////////////////////////////////////////////////
// inspired by link: http://scolton.blogspot.com/2012/09/a-bit-more-kk20-modding.html
// Although I implement PID and super position in http://hefnycopter.net/index.php/developing-source-code/22-quadcopter-control-function-layers.html
void IMU (void)
{
	
		double Alpha;	
		double Beta;
	
	    // calculate ACC-Z
		Alpha = Config.AccParams[Z_INDEX].ComplementaryFilterAlpha / 1000.0;
		Beta = 1- Alpha; // complementary filter to remove noise
		CompAccZ = (double) (Alpha * CompAccZ) + (double) (Beta * Sensors_Latest[ACC_Z_Index]);
		
		// calculate YAW
		Alpha = Config.GyroParams[YAW_INDEX].ComplementaryFilterAlpha / 1000.0;
		Beta = 1- Alpha; // complementary filter to remove noise
		CompGyroZ = (double) (Alpha * CompGyroZ) + (double) (Beta * Sensors_Latest[GYRO_Z_Index]);
		
		Alpha = Config.GyroParams[PITCH_INDEX].ComplementaryFilterAlpha / 1000.0;
		Beta = 1- Alpha; // complementary filter to remove noise
		CompGyroPitch = (double) (Alpha * CompGyroPitch) + (double) (Beta * Sensors_Latest[GYRO_PITCH_Index]);
		Alpha = Config.GyroParams[ROLL_INDEX].ComplementaryFilterAlpha / 1000.0;
		Beta = 1- Alpha; // complementary filter to remove noise
		CompGyroRoll  = (double) (Alpha * CompGyroRoll)  + (double) (Beta * Sensors_Latest[GYRO_ROLL_Index]);
		
					
		// GYRO Always calculated.
		gyroPitch =	PID_Calculate (Config.GyroParams[PITCH_INDEX],	&PID_GyroTerms[PITCH_INDEX],CompGyroPitch);	
		gyroRoll  = PID_Calculate (Config.GyroParams[ROLL_INDEX],	&PID_GyroTerms[ROLL_INDEX],CompGyroRoll); 
		gyroYaw   = PID_Calculate (Config.GyroParams[YAW_INDEX],	&PID_GyroTerms[YAW_INDEX],CompGyroZ -((double)((float)RX_Snapshot[RXChannel_RUD]  * 0.5f))); 
	
			
		// Read ACC and Trims
		// ACC directions are same as GYRO direction [we added "-" for this purpose] 
		double APitch = - Sensors_Latest[ACC_PITCH_Index] - Config.Acc_Pitch_Trim;
		double ARoll  = - Sensors_Latest[ACC_ROLL_Index]  - Config.Acc_Roll_Trim;
		double DT_YAW =  (double)Sensors_Latest[GYRO_Z_Index] * GYRO_RATE * TimeDef * 0.001; 
		
		if ( TCNT1H > TCNT1H_OLD) 
		{
			TimeDef = TCNT1H - TCNT1H_OLD;
		}
		else
		{
			TimeDef = 0xffff - TCNT1H_OLD + TCNT1H ;
		}
		TCNT1H_OLD += TimeDef;
		
		//// Do the Magic of IMU LEVELING here
		//// check also : http://scolton.blogspot.com/2012/09/fun-with-complementary-filter-multiwii.html
		AnglePitch = AnglePitch
				   + (double)Sensors_Latest[GYRO_PITCH_Index] * GYRO_RATE * TimeDef * 0.001
					   + (sin(AngleRoll * DEG_TO_RAD) * DT_YAW)  // integrate component of yaw rate into pitch angle
						;
		AngleRoll = AngleRoll  
				  + (double)Sensors_Latest[GYRO_ROLL_Index]  * GYRO_RATE * TimeDef * 0.001
					  - (sin(AnglePitch * DEG_TO_RAD) * DT_YAW)  // integrate component of yaw rate into roll angle
					   ; 
		//RotateV();	
		// Correct Drift using ACC
		//Alpha = Config.AccParams[PITCH_INDEX].ComplementaryFilterAlpha / 1000.0; // TODO: optimize
		//Beta = 1- Alpha;
		//#define ACC_SMALL_ANGLE	40
		//// if small angle then correct using ACC
		//if ((APitch < ACC_SMALL_ANGLE) && (APitch > -ACC_SMALL_ANGLE)) 
		//{
			//AnglePitch = Alpha * AnglePitch + Beta * APitch  * 10;
		//}
		//
		//Alpha = Config.AccParams[ROLL_INDEX].ComplementaryFilterAlpha / 1000.0; // TODO: optimize
		//Beta = 1- Alpha;
		//if ((ARoll  < ACC_SMALL_ANGLE) && (ARoll  > -ACC_SMALL_ANGLE))
		//{
			//AngleRoll =  Alpha * AngleRoll + Beta * ARoll * 10 ;
			//AngleZ =  Alpha * AngleZ + Beta * CompAccZ * 10;
//
		//}
		//
			
		////////// Attitude of the estimated vector
		////////int32_t sqGZ = AngleZ * AngleZ;
		////////int32_t sqGRoll = AngleRoll * AngleRoll;
		////////int32_t sqGPitch = AnglePitch * AnglePitch;
		////////int32_t sqGRoll_sqGZ = sqGRoll + sqGZ;
		////////float invmagXZ  = InvSqrt(sqGRoll_sqGZ);
		//////////invG = InvSqrt(sqGRoll_sqGZ + sqGPitch);
		////////NavX = _atan2(AngleRoll , AngleZ) / 100.0f;
		////////NavY = _atan2(AnglePitch , invmagXZ*sqGRoll_sqGZ) / 100.0f;
		////////
		return ; /// FOR TESTING
	
			//
		// multiwii HEADFREE ... compact formula
/*
	if(f.HEADFREE_MODE) 
	{ //to optimize
		float radDiff = (heading - headFreeModeHold) * 0.0174533f; // where PI/180 ~= 0.0174533
		float cosDiff = cos(radDiff);
		float sinDiff = sin(radDiff);
		int16_t rcCommand_PITCH = rcCommand[PITCH]*cosDiff + rcCommand[ROLL]*sinDiff;
		rcCommand[ROLL] =  rcCommand[ROLL]*cosDiff - rcCommand[PITCH]*sinDiff; 
		rcCommand[PITCH] = rcCommand_PITCH;
	}
*/	
		// BUG:
		// you should * by 0.7 in case of different directions between board nad flying mode
		// this is to distribute sticks values and has nothing to do with *0.5 that u can keep.	
		if ((Config.BoardOrientationMode==QuadFlyingMode_PLUS) && (Config.QuadFlyingMode==QuadFlyingMode_X))
		{
			if (Config.FrameType == FRAMETYPE_QUADCOPTER)
			{	
				NavY += ( -  (double)((float)RX_Snapshot[RXChannel_AIL]  * 0.7f));
				NavY += ( -  (double)((float)RX_Snapshot[RXChannel_ELE]  * 0.7f));	
				NavX += ( -  (double)((float)RX_Snapshot[RXChannel_AIL]  * 0.7f));
				NavX += ( +  (double)((float)RX_Snapshot[RXChannel_ELE]  * 0.7f));	
			}
			else
			{	// if TRI Copter Fly in A mode.
				NavY += ( +  (double)((float)RX_Snapshot[RXChannel_ELE] * 0.7f));	
				NavX += ( +  (double)((float)RX_Snapshot[RXChannel_AIL] * 0.7f));
			}				
		}
		else if (Config.BoardOrientationMode==Config.QuadFlyingMode)  // ver 0.9.9 code optimization
		{ // for [BoardOrientationMode==QuadFlyingMode_X] this is considered invalid for Tricopter
				NavY += ( -  (double)((float)RX_Snapshot[RXChannel_ELE] * 0.7f));	
				NavX += ( -  (double)((float)RX_Snapshot[RXChannel_AIL] * 0.7f));
		}
		else if ((Config.BoardOrientationMode==QuadFlyingMode_X) && (Config.QuadFlyingMode==QuadFlyingMode_PLUS))
		{  // !!INVALID mode for TRICOPTER
				NavY += ( +  (double)((float)RX_Snapshot[RXChannel_AIL]  * 0.7f));
				NavY += ( -  (double)((float)RX_Snapshot[RXChannel_ELE]  * 0.7f));	
				NavX += ( -  (double)((float)RX_Snapshot[RXChannel_AIL]  * 0.7f));
				NavX += ( -  (double)((float)RX_Snapshot[RXChannel_ELE]  * 0.7f));	
		}
			
		if (IS_FLYINGMODE_ACRO==0)
		{
			
			gyroPitch+=	PID_Calculate_ACC (Config.AccParams[PITCH_INDEX], &PID_AccTerms[PITCH_INDEX],NavY); //AnglePitch);	
			gyroRoll += PID_Calculate_ACC (Config.AccParams[ROLL_INDEX], &PID_AccTerms[ROLL_INDEX],NavX); //AngleRoll); 
		 
		}
		
	
}


	

	
double IMU_HeightKeeping ()
{
	static bool bALTHOLD = false;
	static int16_t ThrottleTemp = 0;
	static int16_t ThrottleZERO = 0;
	static int8_t	IgnoreTimeOut=0;
	
	double Temp;
//	ThrottleTemp = RX_Snapshot[RXChannel_THR];
	
	// calculate damping
	
	Landing = PID_Calculate (Config.AccParams[Z_INDEX], &PID_AccTerms[Z_INDEX],-CompAccZ) ;
			
			
	// Calculate Altitude Hold
	if ((Config.RX_mode==RX_mode_UARTMode) && (IS_MISC_SENSOR_SONAR_ENABLED==true) && (nFlyingModes == FLYINGMODE_ALTHOLD))
	{
		RX_SONAR_TRIGGER = HIGH;
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{	
			Temp = RX_SONAR_RAW; 
		}
	
		if (Temp < 550) // if SONAR Reading is VALID - not BEYOND maximum range
		{
			
			if ((bALTHOLD == false))
			{   
				if (ThrottleTemp<3)
				{ // current altitude is the old one so skip readings till update.
					ThrottleTemp+=1;
					return Landing ;
				}
				// first time to switch to ALTHOLD
				LastAltitudeHold = Temp; // measure Altitude
				PID_SonarTerms[0].I=0;   // ZERO I
				bALTHOLD = true;
			}
			
			AltDiff = LastAltitudeHold - Temp;
			if ((AltDiff<50) && (AltDiff>-50)) // no sudden change or false read
			{
				IgnoreTimeOut=0;
				ThrottleTemp = PID_Calculate (Config.SonarParams[0], &PID_SonarTerms[0],AltDiff) ;	
				if (AltDiff==0) 
				{
					ThrottleZERO = ThrottleTemp;
						
				}
			}
										
			Landing += ThrottleTemp;
		}
		else
		{
			Landing += ThrottleZERO;
		}
		
		RX_SONAR_TRIGGER = LOW;
	}
	else
	{
			ThrottleTemp=0;
			bALTHOLD=false;
	}
	
	
	return Landing;
}