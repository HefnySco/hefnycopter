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



//
//// inspired by to Multiwii
//// 
//void RotateV ()
//{
	///*
	//void rotateV(struct fp_vector *v,float* delta) {
	  //fp_vector v_tmp = *v;
	  //v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
	  //v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
	  //v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X; 
	//*/
	//
	//AngleZ     -= ((Sensors_Latest[GYRO_ROLL_Index]  * AngleRoll ) + ( Sensors_Latest[GYRO_PITCH_Index] * AnglePitch)) * GYRO_RATE;
	//AngleRoll  += ((Sensors_Latest[GYRO_ROLL_Index]  * AngleZ )    - ( Sensors_Latest[GYRO_Z_Index]     * AnglePitch)) * GYRO_RATE;
	//AnglePitch += ((Sensors_Latest[GYRO_PITCH_Index] * AngleZ )	   + ( Sensors_Latest[GYRO_Z_Index]	    * AngleRoll )) * GYRO_RATE;
//}
//


void IMU_Reset()
{
	
	AnglePitch=0;
	AngleRoll=0;
	
}
//////////////////////////////////////////////////////////////////////////
// inspired by link: http://scolton.blogspot.com/2012/09/a-bit-more-kk20-modding.html
// Although I implement PID and super position in http://hefnycopter.net/index.php/developing-source-code/22-quadcopter-control-function-layers.html
void IMU (void)
{
	
		double Alpha;	
		double Beta;
	
	    // calculate ACC-Z
		Alpha = Config.AccParams[1].ComplementaryFilterAlpha / 1000.0;
		Beta = 1- Alpha; // complementary filter to remove noise
		CompAccZ = (double) (Alpha * CompGyroZ) + (double) (Beta * Sensors_Latest[ACC_Z_Index]);
		
		// calculate YAW
		Alpha = Config.GyroParams[1].ComplementaryFilterAlpha / 1000.0;
		Beta = 1- Alpha; // complementary filter to remove noise
		CompGyroZ = (double) (Alpha * CompGyroZ) + (double) (Beta * Sensors_Latest[GYRO_Z_Index]);
		
		Alpha = Config.GyroParams[0].ComplementaryFilterAlpha / 1000.0;
		Beta = 1- Alpha; // complementary filter to remove noise
		CompGyroPitch = (double) (Alpha * CompGyroPitch) + (double) (Beta * Sensors_Latest[GYRO_PITCH_Index]);
		CompGyroRoll  = (double) (Alpha * CompGyroRoll)  + (double) (Beta * Sensors_Latest[GYRO_ROLL_Index]);
		
					
		// GYRO Always calculated.
		gyroPitch =	PID_Calculate (Config.GyroParams[0], &PID_GyroTerms[0],CompGyroPitch);	
		gyroRoll  = PID_Calculate (Config.GyroParams[0], &PID_GyroTerms[1],CompGyroRoll); 
		gyroYaw   = PID_Calculate (Config.GyroParams[1], &PID_GyroTerms[2],CompGyroZ -((double)((float)RX_Snapshot[RXChannel_RUD]  / 2.0f))); 
	
		//if (IS_FLYINGMODE_ACRO==0)
		//{	// Level or ALT HOLD
			
			// Read ACC and Trims
			// ACC directions are same as GYRO direction [we added "-" for this purpose] 
			double APitch = - Sensors_Latest[ACC_PITCH_Index] - Config.Acc_Pitch_Trim;
			double ARoll  = - Sensors_Latest[ACC_ROLL_Index]  - Config.Acc_Roll_Trim;
			double DT_YAW =  (double)Sensors_Latest[GYRO_YAW_Index] * GYRO_RATE_x_IVR_RAD;
			
			
			// Do the Magic of IMU LEVELING here
			// check also : http://scolton.blogspot.com/2012/09/fun-with-complementary-filter-multiwii.html
			AnglePitch = AnglePitch 
			           + (double)Sensors_Latest[GYRO_PITCH_Index] * GYRO_RATE 
					   + AngleRoll * DT_YAW; // integrate component of yaw rate into pitch angle
			AngleRoll =  AngleRoll  
					   + (double)Sensors_Latest[GYRO_ROLL_Index]  * GYRO_RATE) 
					   - AnglePitch * DT_YAW; // integrate component of yaw rate into roll angle
			
			// Correct Drift using ACC
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
			
			
			
			
			if ((Config.BoardOrientationMode==QuadFlyingMode_PLUS) && (Config.QuadFlyingMode==QuadFlyingMode_X))
			{
				NavY += ( -  (double)((float)RX_Snapshot[RXChannel_AIL]  / 2.0f));
				NavY += ( -  (double)((float)RX_Snapshot[RXChannel_ELE]  / 2.0f));	
				NavX += ( -  (double)((float)RX_Snapshot[RXChannel_AIL]  / 2.0f));
				NavX += ( +  (double)((float)RX_Snapshot[RXChannel_ELE]  / 2.0f));	
			}
			else if ((Config.BoardOrientationMode==QuadFlyingMode_PLUS) && (Config.QuadFlyingMode==QuadFlyingMode_PLUS))
			{
				NavY += ( - (double)((float)RX_Snapshot[RXChannel_ELE] / 2.0f));	
				NavX += ( - (double)((float)RX_Snapshot[RXChannel_AIL] / 2.0f));
			}					
			else if ((Config.BoardOrientationMode==QuadFlyingMode_X) && (Config.QuadFlyingMode==QuadFlyingMode_X))
			{
				NavY += ( - (double)((float)RX_Snapshot[RXChannel_ELE] / 2.0f));	
				NavX += ( - (double)((float)RX_Snapshot[RXChannel_AIL] / 2.0f));
			}
			else if ((Config.BoardOrientationMode==QuadFlyingMode_X) && (Config.QuadFlyingMode==QuadFlyingMode_PLUS))
			{
				NavY += ( +  (double)((float)RX_Snapshot[RXChannel_AIL]  / 2.0f));
				NavY += ( -  (double)((float)RX_Snapshot[RXChannel_ELE]  / 2.0f));	
				NavX += ( -  (double)((float)RX_Snapshot[RXChannel_AIL]  / 2.0f));
				NavX += ( -  (double)((float)RX_Snapshot[RXChannel_ELE]  / 2.0f));	
			}
				
		if (IS_FLYINGMODE_ACRO==0)
		{
			
			gyroPitch+=	PID_Calculate_ACC (Config.AccParams[0], &PID_AccTerms[0],NavY); //AnglePitch);	
			gyroRoll += PID_Calculate_ACC (Config.AccParams[0], &PID_AccTerms[1],NavX); //AngleRoll); 
		 
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
	
	Landing = PID_Calculate (Config.AccParams[1], &PID_AccTerms[2],-CompAccZ) ;
			
			
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
				{ // current altitude is the old one
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
				//else
				//{
					//if (IgnoreTimeOut>200)
					//{
						//// assuming there is a sudden change in ALT which means an obstacle under quad ...eg. table ... so take the new ALT 
						//// as the new measure ,,, thus keep the quad at its absolute level regardless of surface change.
						//LastAltitudeHold = Temp; // measure Altitude
					//}
					//else
					//{
						//IgnoreTimeOut = IgnoreTimeOut +1;
					//}						
				//
				//}
				
							
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