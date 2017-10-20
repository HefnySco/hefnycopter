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



const float PIDFactors[2][3]= {{30.0,4000.0,30.0}, {30.0,4000.0,30.0}};
	
float PID_Calculate (const pid_param_t PID_Params, pid_terms_t *PID_Term, const double  Value, const int8_t PIDFactorSelector)
{
		float Output;
		
		// Calculate Terms 
	    PID_Term->P  = ((float)(Value * PID_Params._P) / PIDFactors[PIDFactorSelector][0]);						
		
		
		
		int16_t DeltaError = (Value - PID_Term->Error);
		
		/*
		// I Logic here:
		// DEAD band = 2
		// Increment or Decrement by Value * PID_Params._I 
		*/
		if ((Value > 2) || (Value < -2))
		{	// only increment I when the Value is increasing compared to the old one, also use [-2,2] as deadband.
			PID_Term->I += (float)((float)(Value * PID_Params._I) / PIDFactors[PIDFactorSelector][1]) ;	// try to replace Value with DeltaError
		}
		//else
		//{
			//PID_Term->I -= (float)(PID_Term->I * PID_I_LEAK_RATE);
		//}	
		
		
		
		PID_Term->D= (float)(DeltaError * PID_Params._D) / PIDFactors[PIDFactorSelector][2] ;
		PID_Term->Error = Value;	
		
				
		// Limit boundaries to custom values defined by user.
		PID_Term->I= Limiterf(PID_Term->I, PID_Params._ILimit);
		PID_Term->P= Limiterf(PID_Term->P, PID_Params._PLimit);
	    PID_Term->D= Limiterf(PID_Term->D, PID_Params._DLimit);
	
		Output = (PID_Term->P + PID_Term->I + PID_Term->D);	// P + I + D
		//Output = Output / 10;

		return  Output; //Limiter(Output,(int16_t)300);
}		

void ZERO_Is()
{
	for (int i=0;i<3;++i)
	{
		PID_GyroTerms[i].I=0;
		PID_AccTerms[i].I=0;
	}
	
	PID_SonarTerms[0].I=0;
}


/*
int16_t PID2_Calculation (pid_param_t PID_Params, pid_terms_t PID_Term, int16_t  Gyro_Value)
{
	int16_t Output;
		
		
   // Calculate Terms 
   
   
    int16_t XAbs = abs(Gyro_Value);
	
	PID_Term->P = (Gyro_Value  * PID_Params._P) / 10;						
	PID_Term->D = abs(Gyro_Value - PID_Term->Error);
	if (abs(Gyro_Value - PID_Term->Error) > abs(Gyro_Value)) // the two parameters are of different signs.
	{
		
		// ignore
	}
	else
	{
		
		if (PID_Term->D)
		{
			
		}
		else
		{
			
		}
	}
	
	PID_Term->Error = Gyro_Value;	
	PID_Term->D2Error = PID_Term->D;
	


}
*/