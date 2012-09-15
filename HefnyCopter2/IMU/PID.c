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



int16_t P2D_Calculate (pid_terms_t PID_Term, double  Gyro_Value, int16_t HumanStick, double CompAngle)
{
		int16_t Output;
		// PITCH
		if ((Gyro_Value>=-1) && (Gyro_Value<=1)) {Gyro_Value=0;}
		PID_Term.P = (Gyro_Value);// - (RX_Latest[RXChannel_ELE] >> 3) );
					

		PID_Term.I = ((HumanStick >> 2) - CompAngle); //CompAngle * Config.GyroParams[0]._I; //PID_Term.I + ((PID_Term.P >>3)  * Config.GyroParams[0]._I);	// Multiply I-term
														// Current Error D-term
		// Differential = New - E[0];
		PID_Term.D= (PID_Term.P - PID_Term.Error)* Config.GyroParams[0]._D;
		PID_Term.Error = PID_Term.P;	
		PID_Term.P = PID_Term.P  * Config.GyroParams[0]._P;						// Multiply P-term 
		
		//PID_Term.D
		PID_Term.I= Limiter(PID_Term.I, Config.GyroParams[0]._ILimit);
		PID_Term.P= Limiter(PID_Term.P, Config.GyroParams[0]._PLimit);
	    PID_Term.D= Limiter(PID_Term.D, Config.GyroParams[0]._DLimit);
	
		Output = PID_Term.P + PID_Term.I + PID_Term.D;	// P + I + D
		Output = Output >> 2;
		return  Limiter(Output,(int16_t)300);
}		