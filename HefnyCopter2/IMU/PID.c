/*
 * PID.c
 *
 * Created: 9/14/2012 6:24:56 AM
 *  Author: hefny
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>


#include "../Include/typedefs.h"
#include "../Include/GlobalValues.h"
#include "../Include/Math.h"
#include "../Include/PID.h"



int16_t P2D_Calculate (pid_param_t PID_Params, pid_terms_t PID_Term, int16_t  Gyro_Value, int16_t HumanStick, int16_t CompAngle)
{
		int16_t Output;
		// PITCH
		if ((Gyro_Value>=-1) && (Gyro_Value<=1)) 
		{
			Gyro_Value=0; 
			return ;
		}
					
		
		PID_Term.I = (CompAngle * PID_Params._I) / 10;						//CompAngle * Config.GyroParams[0]._I; //PID_Term.I + ((PID_Term.P >>3)  * Config.GyroParams[0]._I);	// Multiply I-term
																			// Current Error D-term
		PID_Term.P = (Gyro_Value  * PID_Params._P) / 10;						// Multiply P-term 
		PID_Term.D= ((Gyro_Value - PID_Term.Error)* PID_Params._D) / 10;
		PID_Term.Error = Gyro_Value;	
		
		//PID_Term.D
		PID_Term.I= Limiter(PID_Term.I, PID_Params._ILimit);
		PID_Term.P= Limiter(PID_Term.P, PID_Params._PLimit);
	    PID_Term.D= Limiter(PID_Term.D, PID_Params._DLimit);
	
		Output = PID_Term.P + PID_Term.I + PID_Term.D;	// P + I + D
		return  Limiter(Output,(int16_t)300);
}		