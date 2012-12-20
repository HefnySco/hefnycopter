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









/*
* Positive ANgles are to LEFT & TOP
* inspired by [http://www.starlino.com/imu_guide.html]
*/

//////
//////void IMU_CalculateAngles ()
//////{
  ////////timer = TCNT1;	
  //////gyroXrate = (Sensors_GetGyroRate(GYRO_X_Index)) ;//* 1.0323;//(gyroXadc-gryoZeroX)/Sensitivity - in quids              Sensitivity = 0.00333/3.3*1023=1.0323
  //////gyroYrate = (Sensors_GetGyroRate(GYRO_Y_Index)) ;//* 1.0323;//(gyroYadc-gryoZeroX)/Sensitivity - in quids              Sensitivity = 0.00333/3.3*1023=1.0323
 //////
  //////accXangle = Sensors_Latest[ACC_Y_Index] * 2.08; // output in degree  [ 0 to 314]
  //////accYangle = Sensors_Latest[ACC_X_Index] * 2.08; 
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
 	////accPitch  = Sensors_GetAccAngle(ACC_X_Index);       // in Quids  [0,765]
	////gyroPitch = Sensors_GetGyroRate(GYRO_Y_Index);		// in Quids/seconds
    ////gyroPitch = Kalman_Calculate(0,accPitch, gyroPitch,Sensors_dt);      // calculate filtered Angle
	//////gyroPitch = PID_Calculate(Config.GyroParams[0],PID_Terms[0],gyroPitch ,  RX_Latest[RXChannel_ELE], gyroPitch );
////
	////accRoll   = Sensors_GetAccAngle(ACC_Y_Index);       // in Quids
	////gyroRoll  = Sensors_GetGyroRate(GYRO_X_Index);		// in Quids/seconds
    ////gyroRoll  = Kalman_Calculate(1,accRoll, gyroRoll, Sensors_dt);      // calculate filtered Angle
	//////gyroRoll = PID_Calculate(Config.GyroParams[0],PID_Terms[1],gyroRoll ,  RX_Latest[RXChannel_AIL], gyroRoll );
////
	////
	//////gyroYaw = PID_Calculate(Config.GyroParams[1],PID_Terms[2],Sensors_Latest[GYRO_Z_Index],  RX_Latest[RXChannel_RUD], 0.0);
	////
////}


void IMU_P2D (void)
{
	
		//IMU_CalculateAngles();
		
		// Smoothing using complementary filters
		// for Alpha =0 then Take values with NO FILTERING ... max Alpha is 999
		float Alpha = Config.GyroParams[0].ComplementaryFilterAlpha / 1000;
		float Beta = 1- Alpha;
		
		CompGyroY = (double) (Alpha * CompGyroY) + (double) (Beta * Sensors_Latest[GYRO_Y_Index]);
		CompGyroX = (double) (Alpha * CompGyroX) + (double) (Beta * Sensors_Latest[GYRO_X_Index]);
		
		Alpha = Config.GyroParams[1].ComplementaryFilterAlpha / 1000;
		Beta = 1- Alpha;
		CompGyroZ = (double) (Alpha * CompGyroZ) + (double) (Beta * Sensors_Latest[GYRO_Z_Index]);
		
		
		Alpha = Config.AccParams[0].ComplementaryFilterAlpha / 1000;
		Beta = 1- Alpha;
		CompAccY = (double) (Alpha * CompAccY) + (double) (Beta * (Sensors_Latest[ACC_Y_Index] )); // no Dynamic calibration - ACC_Y_Offset));
		CompAccX = (double) (Alpha * CompAccX) + (double) (Beta * (Sensors_Latest[ACC_X_Index] )); // no Dynamic calibration - ACC_X_Offset));
		
		Alpha = Config.AccParams[1].ComplementaryFilterAlpha / 1000;
		Beta = 1- Alpha;
		CompAccZ = (double) (Alpha * CompAccZ) + (double) (Beta * Sensors_Latest[ACC_Z_Index]);
		//////////////////////////////		
		
		
		// PITCH
		double NavAcc =  -CompAccX ; // - (float)((float)(RX_Snapshot [RXChannel_ELE] - RX_Snapshot_1[RXChannel_ELE])/100.0f);
		gyroPitch = 
				PID_Calculate (Config.GyroParams[0], &PID_GyroTerms[0],CompGyroY)//  -  ((float)RX_Snapshot[RXChannel_ELE] / 32.0f))
			+   PID_Calculate_ACC (Config.AccParams[0], &PID_AccTerms[0],NavAcc );
		
		// ROLL
		 NavAcc =  -CompAccY; // - (float)((float)(RX_Snapshot [RXChannel_AIL] - RX_Snapshot_1[RXChannel_AIL]/100.0f));
		gyroRoll = 
				PID_Calculate (Config.GyroParams[0], &PID_GyroTerms[1],CompGyroX) //)
			+   PID_Calculate_ACC (Config.AccParams[0], &PID_AccTerms[1],NavAcc );
		
		// YAW
		double NavGyro = CompGyroZ - (double)((float)RX_Snapshot[RXChannel_RUD]/4.0f);
		gyroYaw = 
				PID_Calculate (Config.GyroParams[1], &PID_GyroTerms[2],NavGyro); 
		
}


double IMU_HeightKeeping ()
{
	/*
	int16_t Landing = (100 - CompAccZ) ;
	Landing *= Config.AccParams[1]._P; // PID_Terms[2].I not used for YAW 
	Limiter(Landing , Config.AccParams[1]._PLimit);
	*/
	double Landing =0;
	if (CompAccZ > 0)
	{
		PID_AccTerms[2].I=0;
	}
	else 
	{
		Landing = PID_Calculate_ACC (Config.AccParams[1], &PID_AccTerms[2],CompAccZ);
	}
	
	
	return Landing;
	
}