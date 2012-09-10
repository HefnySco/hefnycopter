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
	/*
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
		*/
	return 0;	
}


int16_t Limiter (int16_t Value, int16_t Limit)
{
	if (Value > Limit) return Limit;
	if (Value < -Limit) return -Limit;
}

void IMU_PID (void)
{
		
		
		// PITCH
		term_P[0] = (Sensors_Latest[GYRO_Y_Index] - (RX_Latest[RXChannel_ELE] >> 3) );
		
		term_I[0]= term_I[0] + (term_P[0]  * Config.GyroParams[0]._I);	// Multiply I-term
		term_I[0] = term_I[0] >> 3;
		_Error[0] = term_P[0];						// Current Error D-term
		term_P[0] = term_P[0]  * Config.GyroParams[0]._P;			// Multiply P-term 
		// Differential = _Error[2] - E[3];
		term_D[0]= (_Error[0] - _Error[1])* Config.GyroParams[0]._D;
		term_D[0]= Limiter(term_D[0], Config.GyroParams[0]._DLimit);
		_Error[1] = _Error[0];
		
		//term_D[0]
		term_I[0]= Limiter(term_I[0], Config.GyroParams[0]._ILimit);
		term_P[0]= Limiter(term_P[0], Config.GyroParams[0]._PLimit);
		
		gyroPitch = term_P[0] + term_I[0] + term_D[0];	// P + I + D
		gyroPitch = gyroPitch >> 2;
		gyroPitch = Limiter(gyroPitch,(int16_t)200);
		
		// ROLL
		term_P[1] = (Sensors_Latest[GYRO_X_Index] - (RX_Latest[RXChannel_AIL]  >> 3));
		
		term_I[1] = term_I[1] + (term_P[1] * Config.GyroParams[0]._I);	// Multiply I-term
		term_I[1] = term_I[1] >> 3;
		_Error[2] = term_P[1];						// Current Error D-term
		term_P[1] = term_P[1]  * Config.GyroParams[0]._P;			// Multiply P-term 
		// Differential = _Error[2] - E[3];
		term_D[1]= (_Error[2] - _Error[3]) * Config.GyroParams[0]._D;
		term_D[1]= Limiter(term_D[1], Config.GyroParams[0]._DLimit);
		_Error[3] = _Error[2];
	
		term_I[1]= Limiter(term_I[1], Config.GyroParams[0]._ILimit);
		term_P[1]= Limiter(term_P[1], Config.GyroParams[0]._PLimit);
		
		gyroRoll = term_P[1] + term_I[1] + term_D[1];	// P + I + D
		gyroRoll = gyroRoll >> 2; 
		gyroRoll = Limiter(gyroRoll,(int16_t)200);
		
		// YAW
		term_P[2] = (Sensors_Latest[GYRO_Z_Index] - (RX_Latest[RXChannel_RUD] >> 3));
		
		term_I[2]= term_I[2] + (term_P[2] * Config.GyroParams[1]._I);	// Multiply I-term
		term_I[2] = term_I[2] >> 3;
		_Error[4] = term_P[2];						// Current Error D-term
		term_P[2] = term_P[2]  * Config.GyroParams[1]._P;			// Multiply P-term 
		// Differential = _Error[2] - E[3];
		term_D[2]= (_Error[4] - _Error[5]) * Config.GyroParams[1]._D;
		term_D[2]= Limiter(term_D[2], Config.GyroParams[1]._DLimit);
		_Error[5] = _Error[4];
	
		term_I[2]= Limiter(term_I[2], Config.GyroParams[1]._ILimit);
		term_P[2]= Limiter(term_P[2], Config.GyroParams[1]._PLimit);
		
		gyroYaw = term_P[2] + term_I[2] + term_D[2];	// P + I + D
		gyroYaw = gyroYaw >>2;
		gyroYaw = Limiter(gyroYaw,(int16_t)200);
}