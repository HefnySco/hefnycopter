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


void IMU_CalculateAngles ()
{
  //timer = TCNT1;	
  gyroXrate = (Sensors_Latest[GYRO_X_Index]) ;//* 1.0323;//(gyroXadc-gryoZeroX)/Sensitivity - in quids              Sensitivity = 0.00333/3.3*1023=1.0323
  gyroYrate = (Sensors_Latest[GYRO_Y_Index]) ;//* 1.0323;//(gyroYadc-gryoZeroX)/Sensitivity - in quids              Sensitivity = 0.00333/3.3*1023=1.0323
 
  accXangle = Sensors_Latest[ACC_X_Index] * 2.08; // output in degree  [ 0 to 314]
  accYangle = Sensors_Latest[ACC_Y_Index] * 2.08; 
  accZangle = Sensors_Latest[ACC_Z_Index] * 2.08; // maximum value means horizontal [in steady state]
 
  
  CompAngleX = (0.6*(CompAngleX+(gyroXrate)*Sensors_dt  / 10000 ))-(0.4*(accYangle));  // 0.0032 is imperical value based on comparing gyroYangle with accXangle
  CompAngleY = (0.6*(CompAngleY-(gyroYrate)*Sensors_dt  / 10000 ))-(0.4*(accXangle));
  CompAngleZ = (0.6*(CompAngleZ-(gyroZrate)*Sensors_dt  / 10000 ))-(0.4*(accZangle));
  
  
} 


/*
*  Positive Angles are to Right & Bottom.
*/




void IMU_Kalman (void)
{
 	accPitch  = Sensors_GetAccAngle(ACC_X_Index);       // in Quids  [0,765]
	gyroPitch = Sensors_GetGyroRate(GYRO_Y_Index);		// in Quids/seconds
    gyroPitch = Kalman_Calculate(0,accPitch, gyroPitch,Sensors_dt);      // calculate filtered Angle
	gyroPitch = P2D_Calculate(Config.GyroParams[0],PID_Terms[0],gyroPitch ,  RX_Latest[RXChannel_ELE], gyroPitch );

	accRoll   = Sensors_GetAccAngle(ACC_Y_Index);       // in Quids
	gyroRoll  = Sensors_GetGyroRate(GYRO_X_Index);		// in Quids/seconds
    gyroRoll  = Kalman_Calculate(1,accRoll, gyroRoll, Sensors_dt);      // calculate filtered Angle
	gyroRoll = P2D_Calculate(Config.GyroParams[0],PID_Terms[1],gyroRoll ,  RX_Latest[RXChannel_AIL], gyroRoll );

	
	gyroYaw = P2D_Calculate(Config.GyroParams[1],PID_Terms[2],Sensors_Latest[GYRO_Z_Index],  RX_Latest[RXChannel_RUD], 0.0);
	
}

void IMU_P2D (void)
{
		IMU_CalculateAngles();
		
		// PITCH
		gyroPitch = P2D_Calculate(Config.GyroParams[0], PID_Terms[0],Sensors_Latest[GYRO_Y_Index], RX_Latest[RXChannel_ELE], (Sensors_Latest[ACC_Y_Index]));
		// ROLL
		gyroRoll = P2D_Calculate(Config.GyroParams[0], PID_Terms[1],Sensors_Latest[GYRO_X_Index], RX_Latest[RXChannel_AIL], (Sensors_Latest[ACC_X_Index]));
		// YAW
		gyroYaw = P2D_Calculate(Config.GyroParams[1], PID_Terms[2],Sensors_Latest[GYRO_Z_Index], RX_Latest[RXChannel_RUD],0.0);
		
}