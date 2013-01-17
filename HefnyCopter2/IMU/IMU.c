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







/*
* Positive ANgles are to LEFT & TOP
* inspired by [http://www.starlino.com/imu_guide.html]
*/

//////
//////void IMU_CalculateAngles ()
//////{
  ////////timer = TCNT1;	
  //////gyroXrate = (Sensors_GetGyroRate(GYRO_ROLL_Index)) ;//* 1.0323;//(gyroXadc-gryoZeroX)/Sensitivity - in quids              Sensitivity = 0.00333/3.3*1023=1.0323
  //////gyroYrate = (Sensors_GetGyroRate(GYRO_PITCH_Index)) ;//* 1.0323;//(gyroYadc-gryoZeroX)/Sensitivity - in quids              Sensitivity = 0.00333/3.3*1023=1.0323
 //////
  //////accXangle = Sensors_Latest[ACC_ROLL_Index] * 2.08; // output in degree  [ 0 to 314]
  //////accYangle = Sensors_Latest[ACC_PITCH_Index] * 2.08; 
  //////accZangle = Sensors_Latest[ACC_Z_Index] * 2.08; // maximum value means horizontal [in steady state]
 //////
  //////
  //////CompAngleX = (0.98 *(CompAngleX+(gyroXrate)*Sensors_dt  / 1000000 /50 )) -(0.0416*(accXangle));  // 0.0032 is imperical value based on comparing gyroYangle with accXangle [2.08 * 0.02] = 0.0416
  //////CompAngleY = (0.98 *(CompAngleY+(gyroYrate)*Sensors_dt  / 1000000 /50 )) -(0.0416*(accYangle));
  //////CompAngleZ = (0.98 *(CompAngleZ+(gyroZrate)*Sensors_dt  / 1000000 /50 )) -(0.0416*(accZangle));
  //////
  //////
//////} 


/*
*  Positive Angles are to Right & Bottom.
*/




////void IMU_Kalman (void)
////{
 	////accPitch  = Sensors_GetAccAngle(ACC_PITCH_Index);       // in Quids  [0,765]
	////gyroPitch = Sensors_GetGyroRate(GYRO_PITCH_Index);		// in Quids/seconds
    ////gyroPitch = Kalman_Calculate(0,accPitch, gyroPitch,Sensors_dt);      // calculate filtered Angle
	//////gyroPitch = PID_Calculate(Config.GyroParams[0],PID_Terms[0],gyroPitch ,  RX_Latest[RXChannel_ELE], gyroPitch );
////
	////accRoll   = Sensors_GetAccAngle(ACC_ROLL_Index);       // in Quids
	////gyroRoll  = Sensors_GetGyroRate(GYRO_ROLL_Index);		// in Quids/seconds
    ////gyroRoll  = Kalman_Calculate(1,accRoll, gyroRoll, Sensors_dt);      // calculate filtered Angle
	//////gyroRoll = PID_Calculate(Config.GyroParams[0],PID_Terms[1],gyroRoll ,  RX_Latest[RXChannel_AIL], gyroRoll );
////
	////
	//////gyroYaw = PID_Calculate(Config.GyroParams[1],PID_Terms[2],Sensors_Latest[GYRO_Z_Index],  RX_Latest[RXChannel_RUD], 0.0);
	////
////}

//// Complementary Filter Implementation #3:
//filtered_angle = filtered_angle + gyro_rate*dt;
//error_angle = acc_angle - filtered_angle;
//filtered_angle = filtered_angle + (1-A)*error_angle;
//

void IMU (void)
{
	
		double Alpha;	
		double Beta;
	
		if (nFlyingModes == FLYINGMODE_ACRO)
		{
			Alpha = Config.GyroParams[0].ComplementaryFilterAlpha / 1000.0;
			Beta = 1- Alpha;
			CompGyroPitch = (double) (Alpha * CompGyroPitch) + (double) (Beta * Sensors_Latest[GYRO_PITCH_Index]);
			CompGyroRoll  = (double) (Alpha * CompGyroRoll)  + (double) (Beta * Sensors_Latest[GYRO_ROLL_Index]);
			
			gyroPitch =	PID_Calculate (Config.GyroParams[0], &PID_GyroTerms[0],CompGyroPitch);	
			gyroRoll  = PID_Calculate (Config.GyroParams[0], &PID_GyroTerms[1],CompGyroRoll); 
		}
		else
		{	
			
			// Read ACC and Trims 
			double APitch = - Sensors_Latest[ACC_PITCH_Index] - Config.Acc_Pitch_Trim;
			double ARoll  = - Sensors_Latest[ACC_ROLL_Index]  - Config.Acc_Roll_Trim;
			
			
			
			// ACC LPF
			Alpha = Config.AccParams[0].ComplementaryFilterAlpha / 1000.0;
			Beta = 1- Alpha;
			CompGyroPitch = (double) (Alpha * CompGyroPitch) + (double) (Beta * Sensors_Latest[GYRO_PITCH_Index]);
			//gyroYangle += Sensors_Latest[GYRO_PITCH_Index] * 0.042;
			
			CompGyroRoll  = (double) (Alpha * CompGyroRoll)  + (double) (Beta * Sensors_Latest[GYRO_ROLL_Index]);
			//gyroXangle += Sensors_Latest[GYRO_ROLL_Index]  * 0.042;
			
			CompAccPitch  = (double) (Alpha * CompAccPitch)  + (double) (Beta *  APitch);
			CompAccRoll   = (double) (Alpha * CompAccRoll)   + (double) (Beta *  ARoll );
			
			
			Alpha = Config.AccParams[1].ComplementaryFilterAlpha / 1000.0;
			Beta = 1- Alpha;
			CompAccZ = (double) (Alpha * CompAccZ) + (double) (Beta * (double)Sensors_Latest[ACC_Z_Index]);
			
			
			// Calculate tilt angle with ACC.
			// CompAccPitch from [-120,120] === [-90,90]deg
				
			if (((CompAccZ <= 60) && (CompAccZ >=-60)) 
			&& ((gyroYangle <= 40) && (gyroYangle >= -40)) 
			&& ((gyroXangle <= 40) && (gyroXangle >= -40)))
			{
				//double Tau = (Config.GyroParams[0].ComplementaryFilterAlpha / (1000.0 - Config.GyroParams[0].ComplementaryFilterAlpha)) * 0.042;
				//AnglePitch = Tau * CompGyroPitch + CompAccPitch;
				//AngleRoll  = Tau * CompGyroRoll  + CompAccRoll;	
				AnglePitch = 0.97 * (AnglePitch + (double)Sensors_Latest[GYRO_PITCH_Index] * 0.042) + 0.03 * APitch;
				AngleRoll =  0.97 * (AngleRoll  + (double)Sensors_Latest[GYRO_ROLL_Index]  * 0.042) + 0.03 * ARoll;
			}	
		
			NavY = AnglePitch;
			NavX = AngleRoll;
			
			if ((Config.BoardOrientationMode==QuadFlyingMode_PLUS) && (Config.QuadFlyingMode==QuadFlyingMode_X))
			{
				NavY += ( -  (double)((float)RX_Snapshot[RXChannel_AIL]  / 3.0f));
				NavY += ( -  (double)((float)RX_Snapshot[RXChannel_ELE]  / 3.0f));	
				NavX += ( -  (double)((float)RX_Snapshot[RXChannel_AIL]  / 3.0f));
				NavX += ( +  (double)((float)RX_Snapshot[RXChannel_ELE]  / 3.0f));	
			}
			else if ((Config.BoardOrientationMode==QuadFlyingMode_PLUS) && (Config.QuadFlyingMode==QuadFlyingMode_PLUS))
			{
				NavY += ( - (double)((float)RX_Snapshot[RXChannel_ELE] / 3.0f));	
				NavX += ( - (double)((float)RX_Snapshot[RXChannel_AIL] / 3.0f));
			}					
			else if ((Config.BoardOrientationMode==QuadFlyingMode_X) && (Config.QuadFlyingMode==QuadFlyingMode_X))
			{
				NavY += ( - (double)((float)RX_Snapshot[RXChannel_ELE] / 3.0f));	
				NavX += ( - (double)((float)RX_Snapshot[RXChannel_AIL] / 3.0f));
		
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
		gyroYaw = PID_Calculate (Config.GyroParams[1], &PID_GyroTerms[2],CompGyroZ -((double)((float)RX_Snapshot[RXChannel_RUD]  / 3.0f))); 
	
}


double IMU_HeightKeeping ()
{
	/*
	int16_t Landing = (100 - CompAccZ) ;
	Landing *= Config.AccParams[1]._P; // PID_Terms[2].I not used for YAW 
	Limiter(Landing , Config.AccParams[1]._PLimit);
	*/
	double Landing =0;
	if ((CompAccZ > 2) || (CompAccZ < -2))
	{
		Landing = PID_Calculate_ACC (Config.AccParams[1], &PID_AccTerms[2],-CompAccZ);
	}
	
	
	return Landing;
	
}