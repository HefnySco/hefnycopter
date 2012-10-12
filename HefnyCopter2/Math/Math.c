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



int16_t Limiter (int16_t Value, int16_t Limit)
{
	if (abs(Value) > Limit) return Limit;
	
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
