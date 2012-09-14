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
 
  
  CompAngleX = (0.6*(CompAngleX+(gyroXrate)*2/1000))-(0.4*(accYangle));
  CompAngleY = (0.6*(CompAngleY-(gyroYrate)*2/1000))-(0.4*(accXangle));
} 


/*
*  Positive Angles are to Right & Bottom.
*/



uint16_t LastLoopTime;
void IMU_Kalman (void)
{
	int ACC_angle = Sensors_GetAccAngle(ACC_X_Index);                                           // in Quids
	int GYRO_rate = Sensors_GetGyroRate(GYRO_Y_Index);												// in Quids/seconds
    gyroPitch = kalmanCalculate(0,ACC_angle, GYRO_rate, LastLoopTime - TCNT1_X_GlobalTimer);      // calculate filtered Angle

	ACC_angle = Sensors_GetAccAngle(ACC_Y_Index);                                           // in Quids
	GYRO_rate = Sensors_GetGyroRate(GYRO_X_Index);												// in Quids/seconds
    gyroRoll = kalmanCalculate(0,ACC_angle, GYRO_rate, LastLoopTime - TCNT1_X_GlobalTimer);      // calculate filtered Angle

	LastLoopTime = TCNT1_X_GlobalTimer; // in 100 us unit
	
}

void IMU_PID (void)
{
		// PITCH
		if ((Sensors_Latest[GYRO_Y_Index]>=-1) && (Sensors_Latest[GYRO_Y_Index]<=1)) {Sensors_Latest[GYRO_Y_Index]=0;}
		term_P[0] = (Sensors_Latest[GYRO_Y_Index]);// + CompAngleY; // - (RX_Latest[RXChannel_ELE] >> 3) );

		term_I[0] = -CompAngleY * Config.GyroParams[0]._I; //term_I[0] + ((term_P[0] >>3)  * Config.GyroParams[0]._I);	// Multiply I-term
														// Current Error D-term
		// Differential = New - E[0];
		term_D[0]= (term_P[0] - _Error[0])* Config.GyroParams[0]._D;
		_Error[0] = term_P[0];	
		term_P[0] = term_P[0]  * Config.GyroParams[0]._P;						// Multiply P-term 
		
		//term_D[0]
		term_I[0]= Limiter(term_I[0], Config.GyroParams[0]._ILimit);
		term_P[0]= Limiter(term_P[0], Config.GyroParams[0]._PLimit);
	    term_D[0]= Limiter(term_D[0], Config.GyroParams[0]._DLimit);
	
		gyroPitch = term_P[0] + term_I[0] + term_D[0];	// P + I + D
		gyroPitch = gyroPitch >> 2;
		gyroPitch = Limiter(gyroPitch,(int16_t)300);
		
		// ROLL
		
		if ((Sensors_Latest[GYRO_X_Index]>=-1) && (Sensors_Latest[GYRO_X_Index]<=1)) {Sensors_Latest[GYRO_X_Index]=0;}
		term_P[1] = (Sensors_Latest[GYRO_X_Index]); // -CompAngleX; // - (RX_Latest[RXChannel_AIL]  >> 3));
		
		term_I[1] = -CompAngleX * Config.GyroParams[0]._I; //term_I[1] + ((term_P[1] >>3 ) * Config.GyroParams[0]._I);	// Multiply I-term

		// Differential = NEW - E[1];
		term_D[1]= (term_P[1] - _Error[1]) * Config.GyroParams[0]._D;
		_Error[1] = term_P[1];						// Current Error D-term
		term_P[1] = term_P[1]  * Config.GyroParams[0]._P;			// Multiply P-term 
	
		term_I[1]= Limiter(term_I[1], Config.GyroParams[0]._ILimit);
		term_P[1]= Limiter(term_P[1], Config.GyroParams[0]._PLimit);
		term_D[1]= Limiter(term_D[1], Config.GyroParams[0]._DLimit);
		
		gyroRoll = term_P[1] + term_I[1] + term_D[1];	// P + I + D
		gyroRoll = gyroRoll >> 2; 
		gyroRoll = Limiter(gyroRoll,(int16_t)300);
		
		// YAW
		if ((Sensors_Latest[GYRO_Z_Index]>=-1) && (Sensors_Latest[GYRO_Z_Index]<=1)) {Sensors_Latest[GYRO_Z_Index]=0;}
		term_P[2] = (Sensors_Latest[GYRO_Z_Index]) ;// - (RX_Latest[RXChannel_RUD] >> 3));
		
		term_I[2]= term_I[2] + ((term_P[2] >> 3) * Config.GyroParams[1]._I);	// Multiply I-term

		// Differential = NEW - E[2];
		term_D[2]= (term_P[2] - _Error[2]) * Config.GyroParams[1]._D;
		_Error[2] = term_P[2];										// Current Error D-term
		term_P[2] = term_P[2]  * Config.GyroParams[1]._P;			// Multiply P-term 
		
		term_I[2]= Limiter(term_I[2], Config.GyroParams[1]._ILimit);
		term_P[2]= Limiter(term_P[2], Config.GyroParams[1]._PLimit);
		term_D[2]= Limiter(term_D[2], Config.GyroParams[1]._DLimit);
		
		gyroYaw = term_P[2] + term_I[2] + term_D[2];	// P + I + D
		gyroYaw = gyroYaw >> 2;
		gyroYaw = Limiter(gyroYaw,(int16_t)300);
}