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
#include "../Include/IMU.h"



//#define aX A3
//#define aY A4
//#define aZ A5


//gyros
float gyroXadc;
float gyroXrate;
float gyroXangle;

float gyroYadc;
float gyroYrate;
float gyroYangle;

float gyroZadc;
float gyroZrate;
float gyroZangle;


//accelerometers

int accZeroX;//x-axis
float accXadc;
float accXval;
float accXangle;

int accZeroY;//y-axis
float accYadc;
float accYval;
float accYangle;

int accZeroZ;//z-axis
float accZadc;
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



void IMU_CalculateAngles ()
{
  //timer = TCNT1;	
  gyroXadc = Sensors_Latest[GYRO_X_Index]/10;
  gyroXrate = (gyroXadc) ;//* 1.0323;//(gyroXadc-gryoZeroX)/Sensitivity - in quids              Sensitivity = 0.00333/3.3*1023=1.0323
  
  if ((gyroXrate<=1) && (gyroXrate>=-1))
  {
	  gyroXrate=0;
  }
  gyroXangle=gyroXangle+gyroXrate * dtime;//Without any filter
  
  gyroYadc = Sensors_Latest[GYRO_Y_Index];
  gyroYrate = (gyroYadc) ;//* 1.0323;//(gyroYadc-gryoZeroX)/Sensitivity - in quids              Sensitivity = 0.00333/3.3*1023=1.0323
  if ((gyroYrate<=1) && (gyroYrate>=-1))
  {
	  gyroYrate=0;
  }
  gyroYangle=gyroYangle+gyroYrate;//Without any filter
  
  gyroZadc = Sensors_Latest[GYRO_Z_Index];
  gyroZrate = (gyroZadc);///1.0323;//(gyroZadc-gryoZeroX)/Sensitivity - in quids              Sensitivity = 0.00333/3.3*1023=1.0323
  //gyroZangle=gyroZangle+gyroZrate*dtime/1000;//Without any filter
  
  accXadc = Sensors_Latest[ACC_X_Index];
  accXval = (accXadc);//(accXadc-accZeroX)/Sensitivity - in quids					  Sensitivity = 0.33/3.3*1023=102,3
  
  accYadc = Sensors_Latest[ACC_Y_Index];
  accYval = (accYadc);///102;//(accXadc-accZeroX)/Sensitivity - in quids              Sensitivity = 0.33/3.3*1023=102,3
  
  accZadc = Sensors_Latest[ACC_Z_Index];
  accZval = (accZadc);///102;//(accXadc-accZeroX)/Sensitivity - in quids              Sensitivity = 0.33/3.3*1023=102,3
  //accZval++;//1g in horizontal position
  
  //R = sqrt(pow(accXval,2)+pow(accYval,2)+pow(accZval,2));//the force vector
  accXangle = accXval * 0.72; //acos(accXval/R)*RAD_TO_DEG-90;
  accYangle = accYval * 0.72; //acos(accYval/R)*RAD_TO_DEG-90;
  //accZangle = acos(accZval/R)*RAD_TO_DEG;
 
  
  CompAngleX = (0.6*(CompAngleX+(gyroXrate)*dt/1000))+(0.4*(accYangle));
  CompAngleY = (0.6*(CompAngleY-(gyroYrate)*dt/1000))+(0.4*(accXangle));
 for (int c=0;c<dt;++c)
 {
   delay_ms(1);
  }  
 // utoa(compAngleX,sXDeg,10);
 
}


int16_t ScaleSensor (int16_t SensorValue, pid_param_t *pid_Param)
{
	
			if ((SensorValue < pid_Param->minSource ) && (SensorValue>- pid_Param->minSource))
			{
				return  0;
			}	
			else
			{
				if ((SensorValue> pid_Param->maxSource)) 
				{
					return pid_Param->maxDest;
				}
				else if ((SensorValue<- pid_Param->maxSource))
				{
					return -pid_Param->maxDest;
				}
				else if (SensorValue > 0) // positive sign
				{
					y = Pitch_Ratio  * (double)(SensorValue - pid_Param->minSource) + pid_Param->minDest;
					return (int16_t) y;
				}
				else if (SensorValue < 0) // negative sign
				{
					y = Pitch_Ratio  * (double)(SensorValue + pid_Param->minSource) - pid_Param->minDest;
					 return (int16_t) y;
				}
		    }	
			
}
