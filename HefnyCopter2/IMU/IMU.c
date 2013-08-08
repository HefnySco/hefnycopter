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



		
//
//// inspired by to Multiwii
//// 
void RotateV ()
{
	///*
	//void rotateV(struct fp_vector *v,float* delta) {
	  //fp_vector v_tmp = *v;
	  //v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
	  //v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
	  //v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X; 
	//*/
	
	double DeltaPitch =  (double)CompGyroPitch	* 0.02;//* GYRO_RATE * TimeDef_m * DEG_TO_RAD;
	double DeltaRoll  =	 (double)CompGyroRoll	* 0.02;//* GYRO_RATE * TimeDef_m * DEG_TO_RAD;
	double DT_YAW	  =  (double)CompGyroZ		* 0.02;//* GYRO_RATE * TimeDef_m * DEG_TO_RAD; 
		
	double AngleRoll_T, AnglePitch_T,AngleZ_T;
	AngleRoll_T = AngleRoll;
	AnglePitch_T = AnglePitch;
	//AngleZ_T = DT_YAW;
	//AngleZ     -= ((DeltaRoll  * AngleRoll_T ) + (DeltaPitch * AnglePitch_T ));
	//AngleRoll  += (DeltaRoll  * AngleZ_T ) - (DT_YAW	* AnglePitch_T ); 
	//AnglePitch += (DeltaPitch * AngleZ_T ) + (DT_YAW	* AngleRoll_T  );
	//AngleZ     += DT_YAW;
	
	// COntrifugal forces does affect rotation
	AngleRoll  += DeltaRoll  - (DT_YAW	* APitch * 0.008064);
	AnglePitch += DeltaPitch + (DT_YAW	* ARoll * 0.008064);  // 0.008064  = 1/124
	
}



void IMU_Reset()
{
	APitch=0;
	ARoll=0;
	AnglePitch=0;
	AngleRoll=0;
	//AngleZ=D90_RAD;//D90_RAD;  // RAD 90 DEG
	
}
//////////////////////////////////////////////////////////////////////////
// inspired by link: http://scolton.blogspot.com/2012/09/a-bit-more-kk20-modding.html
// Although I implement PID and super position in http://hefnycopter.net/index.php/developing-source-code/22-quadcopter-control-function-layers.html
void IMU (void)
{
		double Alpha;	
		double Beta;
		//double tZ = Sensors_Latest[ACC_Z_Index] + Config.AccParams[YAW_INDEX].ComplementaryFilterAlpha / 1000.0;
		double APitchZero;
		double ARollhZero;
	    // calculate ACC-Z
		//Alpha = 0.0;  //Config.AccParams[Z_INDEX].ComplementaryFilterAlpha / 1000.0;
		//Beta = 1.0- Alpha; // complementary filter to remove noise
		//CompAccZ = (double) (Alpha * CompAccZ) + (double) (Beta * (tZ));
		
		// calculate YAW
		Alpha =0.0; // 0.01; //Config.GyroParams[YAW_INDEX].ComplementaryFilterAlpha / 1000.0;
		Beta = 1.0- Alpha; // complementary filter to remove noise
		CompGyroZ = (double) (Alpha * CompGyroZ) + (double) (Beta * Sensors_Latest[GYRO_Z_Index]);
		
		//Alpha = 0.0; //Config.GyroParams[PITCH_INDEX].ComplementaryFilterAlpha / 1000.0;
		//Beta = 1- Alpha; // complementary filter to remove noise
		//CompGyroPitch = (double) (Alpha * CompGyroPitch) + (double) (Beta * Sensors_Latest[GYRO_PITCH_Index]);
		CompGyroPitch = Sensors_Latest[GYRO_PITCH_Index] * 1.0;
		
		//Alpha = 0.0; //Config.GyroParams[ROLL_INDEX].ComplementaryFilterAlpha / 1000.0;
		//Beta = 1- Alpha; // complementary filter to remove noise
		//CompGyroRoll  = (double) (Alpha * CompGyroRoll)  + (double) (Beta * Sensors_Latest[GYRO_ROLL_Index]);
		CompGyroRoll   = Sensors_Latest[GYRO_ROLL_Index] * 1.0;
				
					
		// GYRO Always calculated.
		gyroPitch =	PID_Calculate (Config.GyroParams[PITCH_INDEX],	&PID_GyroTerms[PITCH_INDEX],CompGyroPitch,0);	
		gyroRoll  = PID_Calculate (Config.GyroParams[ROLL_INDEX],	&PID_GyroTerms[ROLL_INDEX],CompGyroRoll,0); 
		gyroYaw   = PID_Calculate (Config.GyroParams[YAW_INDEX],	&PID_GyroTerms[YAW_INDEX],CompGyroZ -((double)((float)RX_Snapshot[RXChannel_RUD]  / 2.0f)),0); 
	
		
		APitchZero = (- Sensors_Latest[ACC_PITCH_Index] - Config.Acc_Pitch_Trim) ;
		ARollhZero = (- Sensors_Latest[ACC_ROLL_Index]  - Config.Acc_Roll_Trim)  ;
		// Read ACC and Trims
		// ACC directions are same as GYRO direction [we added "-" for this purpose] 
		Alpha = 0.5; //Config.GyroParams[PITCH_INDEX].ComplementaryFilterAlpha / 1000.0; // TODO: optimize
		Beta = 1.0- Alpha;
		APitch = Alpha * APitch + Beta * APitchZero;
		ARoll  = Alpha * ARoll  + Beta * ARollhZero;
		
		
		TimeDef = CurrentTCNT1_X - TCNT1H_OLD;
		
		TCNT1H_OLD =CurrentTCNT1_X;
		TimeDef_m = TimeDef ;
		
		
		RotateV();			   
					   
		// Correct Drift using ACC
		#define ACC_SMALL_ANGLE	150.0
		
		// if small angle then correct using ACC
		if (
			((APitch < ACC_SMALL_ANGLE) && (APitch > -ACC_SMALL_ANGLE))
			&&
			((ARoll  < ACC_SMALL_ANGLE) && (ARoll  > -ACC_SMALL_ANGLE))
			)
		{
			Alpha = 0.98; //Config.AccParams[PITCH_INDEX].ComplementaryFilterAlpha / 1000.0; // TODO: optimize
			Beta = 1.0- Alpha;
			AngleRoll =  Alpha * AngleRoll + Beta  * ARollhZero ;
			AnglePitch = Alpha * AnglePitch + Beta *  APitchZero;
		}			
			NavY = AnglePitch; 
			NavX = AngleRoll; 
		
		// multiwii HEADFREE ... compact formula
		double mix[4];

		if ((Config.BoardOrientationMode==QuadFlyingMode_PLUS) && (Config.QuadFlyingMode==QuadFlyingMode_X))
		{
			if (Config.FrameType == FRAMETYPE_QUADCOPTER)
			{	
				mix[0] = 0.7;
				mix[1] = 0.7;
				mix[2] = 0.7;
				mix[3] = -0.7;

				//NavY += ( -  (double)((float)RX_Snapshot[RXChannel_AIL]  * 0.7f));
				//NavY += ( -  (double)((float)RX_Snapshot[RXChannel_ELE]  * 0.7f));	
				//NavX += ( -  (double)((float)RX_Snapshot[RXChannel_AIL]  * 0.7f));
				//NavX += ( +  (double)((float)RX_Snapshot[RXChannel_ELE]  * 0.7f));	
			}
			else
			{	// if TRI Copter Fly in A mode.  [QuadFlyingMode_X === TriFlyingAMode]
				
				//mix = {0.0,-1.0,-1.0,0};
				mix[0] = 0.0;
				mix[1] = -1.0;
				mix[2] = -1.0;
				mix[3] = 0.0;
				
				//NavY += ( +  (double)((float)RX_Snapshot[RXChannel_ELE] ));	
				//NavX += ( +  (double)((float)RX_Snapshot[RXChannel_AIL] ));
			}				
		}
		else if (Config.BoardOrientationMode==Config.QuadFlyingMode)  // ver 0.9.9 code optimization
		{ // for [BoardOrientationMode==QuadFlyingMode_X] this is considered invalid for Tricopter
					//mix = {0.0,1.0,1.0,0};
					mix[0] = 0.0;
					mix[1] = 1.0;
					mix[2] = 1.0;
					mix[3] = 0.0;

					//NavY += ( -  (double)((float)RX_Snapshot[RXChannel_ELE] ));	
					//NavX += ( -  (double)((float)RX_Snapshot[RXChannel_AIL] ));
		}
		else if ((Config.BoardOrientationMode==QuadFlyingMode_X) && (Config.QuadFlyingMode==QuadFlyingMode_PLUS))
		{  // !!INVALID mode for TRICOPTER
					
				//	mix = {-0.7,0.7,0.7,0.7};
						
					mix[0] = -0.7;
					mix[1] = 0.7;
					mix[2] = 0.7;
					mix[3] = 0.7;
	
					//NavY += ( +  (double)((float)RX_Snapshot[RXChannel_AIL]  * 0.7f));
					//NavY += ( -  (double)((float)RX_Snapshot[RXChannel_ELE]  * 0.7f));	
					//NavX += ( -  (double)((float)RX_Snapshot[RXChannel_AIL]  * 0.7f));
					//NavX += ( -  (double)((float)RX_Snapshot[RXChannel_ELE]  * 0.7f));	
		}
					
				NavY += ( -  (double)((float)RX_Snapshot[RXChannel_AIL]  * mix[0]));
				NavY += ( -  (double)((float)RX_Snapshot[RXChannel_ELE]  * mix[1]));	
				NavX += ( -  (double)((float)RX_Snapshot[RXChannel_AIL]  * mix[2]));
				NavX += ( -  (double)((float)RX_Snapshot[RXChannel_ELE]  * mix[3]));	
					
			
		if (IS_FLYINGMODE_ACRO==0)
		{	// STABILIZATION of ALTHOLD
			
			gyroPitch+=	PID_Calculate (Config.AccParams[PITCH_INDEX], &PID_AccTerms[PITCH_INDEX],NavY,1); //AnglePitch);	
			gyroRoll += PID_Calculate (Config.AccParams[ROLL_INDEX], &PID_AccTerms[ROLL_INDEX],NavX,1); //AngleRoll); 
		 
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
	
	Landing = PID_Calculate (Config.AccParams[Z_INDEX], &PID_AccTerms[Z_INDEX],-Sensors_Latest[ACC_Z_Index],0) ;
			
			
	// Calculate Altitude Hold
	if ((Config.RX_mode==RX_mode_SingleMode) && (IS_MISC_SENSOR_SONAR_ENABLED==true) && (nFlyingModes == FLYINGMODE_ALTHOLD))
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
				ThrottleTemp = PID_Calculate (Config.SonarParams[0], &PID_SonarTerms[0],AltDiff,0) ;	
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