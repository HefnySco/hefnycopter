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





//gyros

float gyroXrate;
float gyroXangle;

float gyroYrate;
float gyroYangle;

float gyroZadc;
float gyroZrate;
float gyroZangle;


//accelerometers

int accZeroX;//x-axis
float accXval;
float accXangle;

int accZeroY;//y-axis
float accYval;
float accYangle;

int accZeroZ;//z-axis
float accZval;
float accZangle;
//Results
float xAngle;
float yAngle;
float R;//force vector
//Used for timing
uint16_t timer=0;
double dtime=0;
uint16_t dt;


/*
* Positive ANgles are to LEFT & TOP
* inspired by [http://www.starlino.com/imu_guide.html]
*/

void IMU_CalculateAngles ()
{
  //timer = TCNT1;	
  gyroXrate = (Sensors_Latest[GYRO_X_Index]/10) ;//* 1.0323;//(gyroXadc-gryoZeroX)/Sensitivity - in quids              Sensitivity = 0.00333/3.3*1023=1.0323
  
  if ((gyroXrate<=1) && (gyroXrate>=-1))
  {
	  gyroXrate=0;
  }
  gyroXangle=gyroXangle + gyroXrate;//Without any filter
  
  
  gyroYrate = (Sensors_Latest[GYRO_Y_Index]) ;//* 1.0323;//(gyroYadc-gryoZeroX)/Sensitivity - in quids              Sensitivity = 0.00333/3.3*1023=1.0323
  if ((gyroYrate<=1) && (gyroYrate>=-1))
  {
	  gyroYrate=0;
  }
  gyroYangle=gyroYangle + gyroYrate;//Without any filter
  
  gyroZadc = Sensors_Latest[GYRO_Z_Index];
  gyroZrate = (gyroZadc);///1.0323;//(gyroZadc-gryoZeroX)/Sensitivity - in quids              Sensitivity = 0.00333/3.3*1023=1.0323
  gyroZangle=gyroZangle + gyroZrate; //*dtime/1000;//Without any filter
  
  accXval = Sensors_Latest[ACC_X_Index];//(accXadc-accZeroX)/Sensitivity - in quids					  Sensitivity = 0.33/3.3*1023=102,3
  
  
  accYval = Sensors_Latest[ACC_Y_Index];///102;//(accXadc-accZeroX)/Sensitivity - in quids              Sensitivity = 0.33/3.3*1023=102,3
  
  accZval = Sensors_Latest[ACC_Z_Index];///102;//(accXadc-accZeroX)/Sensitivity - in quids              Sensitivity = 0.33/3.3*1023=102,3
  //accZval++;//1g in horizontal position
  
  //R = sqrt(pow(accXval,2)+pow(accYval,2)+pow(accZval,2));//the force vector
  accXangle = accXval * 0.7563; //acos(accXval/R)*RAD_TO_DEG-90;
  accYangle = accYval * 0.7563; //acos(accYval/R)*RAD_TO_DEG-90;
  accZangle = accZval * 0.7563; //acos(accZval/R)*RAD_TO_DEG;
 
  
  CompAngleX = (0.6*(CompAngleX+(gyroXrate)*2/1000))-(0.4*(accYangle));
  CompAngleY = (0.6*(CompAngleY-(gyroYrate)*2/1000))-(0.4*(accXangle));
  CompAngleZ = (0.6*(CompAngleZ-(gyroZrate)*2/1000))-(0.4*(accZangle));
} 


/*
*  Positive Angles are to Right & Bottom.
*/



uint16_t LastLoopTime[2];
void IMU_Kalman (void)
{
	
   ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
   {
	LastLoopTime[0] = TCNT1_X_GlobalTimer;
   }   
	accPitch  = Sensors_GetAccAngle(ACC_X_Index);       // in Quids
	gyroPitch = Sensors_GetGyroRate(GYRO_Y_Index);		// in Quids/seconds
    gyroPitch = Kalman_Calculate(0,accPitch, gyroRoll, LastLoopTime[0] - LastLoopTime[1]);      // calculate filtered Angle

	accRoll   = Sensors_GetAccAngle(ACC_Y_Index);       // in Quids
	gyroRoll  = Sensors_GetGyroRate(GYRO_X_Index);		// in Quids/seconds
    gyroRoll  = Kalman_Calculate(1,accPitch, gyroRoll, LastLoopTime[0] - LastLoopTime[1]);      // calculate filtered Angle

	
	gyroYaw = P2D_Calculate(PID_Terms[2],GYRO_Z_Index,0.0);
	
	LastLoopTime[1] = LastLoopTime[0]; // in 100 us unit
	
}

void IMU_P2D (void)
{
		IMU_CalculateAngles();
		
		// PITCH
		gyroPitch = P2D_Calculate(PID_Terms[0],GYRO_Y_Index,CompAngleX);
		// ROLL
		gyroRoll = P2D_Calculate(PID_Terms[1],GYRO_X_Index,CompAngleY);
		// YAW
		gyroYaw = P2D_Calculate(PID_Terms[2],GYRO_Z_Index,0.0);
		
}