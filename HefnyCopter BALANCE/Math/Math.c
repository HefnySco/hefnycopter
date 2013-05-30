/*
 * Math.c
 *
 * Created: 14-Sep-12 4:42:44 AM
 *  Author: M.Hefny
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>


#include "../Include/Math.h"


// taken from Multiwii Code
//int16_t _atan2(float y, float x)
//{
  //#define fp_is_neg(val) ((((uint8_t*)&val)[3] & 0x80) != 0)
  //float z = y / x;
  //int16_t zi = abs(int16_t(z * 100)); 
  //int8_t y_neg = fp_is_neg(y);
  //if ( zi < 100 ){
    //if (zi > 10) 
     //z = z / (1.0f + 0.28f * z * z);
   //if (fp_is_neg(x)) {
     //if (y_neg) z -= PI;
     //else z += PI;
   //}
  //} else {
   //z = (PI * 0.5f) - z / (z * z + 0.28f);
   //if (y_neg) z -= PI;
  //}
  //z *= (180.0f / PI * 10); 
  //return z;
//}

int arctan2(int y, int x) {                                    // http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm
   int coeff_1 = 128;                                          // angle in Quids (1024 Quids=360°) <<<<<<<<<<<<<<
   int coeff_2 = 3*coeff_1;
   float abs_y = abs(y)+1e-10;
   float r, angle;

   if (x >= 0) {
     r = (x - abs_y) / (x + abs_y);
     angle = coeff_1 - coeff_1 * r;
   }  else {
     r = (x + abs_y) / (abs_y - x);
     angle = coeff_2 - coeff_1 * r;
   }
   if (y < 0)      return (int)(-angle);
   else            return (int)(angle);
}


float Limiterf (float Value, float Limit)
{
	if (Value > Limit) return   Limit;
	if (Value < -Limit) return -Limit;
	
	return Value;
}

int16_t Limiter (int16_t Value, int16_t Limit)
{
	if (Value > Limit) return   Limit;
	if (Value < -Limit) return -Limit;
	
	return Value;
}


int16_t ScaleSensor (int16_t SensorValue, int16_t minSource, int16_t maxSource, int16_t minDest, int16_t maxDest, double Ratio)
{
	/*
	if ((SensorValue <  minSource ) && (SensorValue>-  minSource))
	{
		return  0;
	}	
	else
	{
		if ((SensorValue>  maxSource)) 
		{
			return  maxDest;
		}
		else if ((SensorValue<-  maxSource))
		{
			return - maxDest;
		}
		else if (SensorValue > 0) // positive sign
		{
			y = Ratio  * (double)(SensorValue -  minSource) +  minDest;
			return (int16_t) y;
		}
		else if (SensorValue < 0) // negative sign
		{
			y = Ratio  * (double)(SensorValue +  minSource) -  minDest;
			 return (int16_t) y;
		}
	   }	
		*/
	return 0;	
}



int16_t _atan2(int32_t y, int32_t x)
{
	float z = (float)y / x;
	int16_t a;
	if ( abs(y) < abs(x) ){
		a = 573 * z / (1.0f + 0.28f * z * z);
		if (x<0) {
			if (y<0) a -= 1800;
			else a += 1800;
		}
	} else {
		a = 900 - 573 * z / (z * z + 0.28f);
		if (y<0) a -= 1800;
	}
	return a;
}

float InvSqrt (float x)
{
	union{
		int32_t i;
		float   f;
	} conv;
	conv.f = x;
	conv.i = 0x5f3759df - (conv.i >> 1);
	return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}
