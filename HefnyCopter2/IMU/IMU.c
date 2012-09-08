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
  gyroXangle=gyroXangle+gyroXrate;//Without any filter
  
  
  gyroYrate = (Sensors_Latest[GYRO_Y_Index]) ;//* 1.0323;//(gyroYadc-gryoZeroX)/Sensitivity - in quids              Sensitivity = 0.00333/3.3*1023=1.0323
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
 
  
  CompAngleX = (0.8*(CompAngleX+(gyroXrate)*2/1000))-(0.2*(accYangle));
  CompAngleY = (0.8*(CompAngleY-(gyroYrate)*2/1000))-(0.2*(accXangle));
} 


/*
*  Positive Angles are to Right & Bottom.
*/

int16_t ScaleSensor (int16_t SensorValue, pid_param_t *pid_Param, double Ration)
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
			y = Ration  * (double)(SensorValue - pid_Param->minSource) + pid_Param->minDest;
			return (int16_t) y;
		}
		else if (SensorValue < 0) // negative sign
		{
			y = Ration  * (double)(SensorValue + pid_Param->minSource) - pid_Param->minDest;
			 return (int16_t) y;
		}
	   }	
			
}


void IMU_PID (void)
{
		int16_t term_P, term_I, term_D;	
		term_P = (RX_Latest[RXChannel_ELE] + Sensors_Latest[GYRO_Y_Index]);
		
		term_I= (term_I + term_P) * _I[0];	// Multiply I-term
		
		
		_E[0] = term_P;						// Current Error D-term
	
		term_P = term_P  * _P[0];			// Multiply P-term 
	
		// Differential = _E[2] - E[3];
		term_D= (_E[0] - _E[1]) * _D[0];
	
		_E[1] = _E[0];
	
	
		gyroPitch = term_P + term_I + term_D;	// P + I + D
		
}