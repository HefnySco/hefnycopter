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





float PID_Calculate_ACC (const pid_param_t PID_Params, pid_terms_t *PID_Term, const double  Value)
{
	#define ACC_I_MIN	4
		
	float Output;
		
		double AbsValue = abs (Value);
		// Calculate Terms 
		if (AbsValue > 1)
	    PID_Term->P  = ((float)(Value * PID_Params._P) / 10.0f);						
		
		
		//double AbsValue = abs (Value);
		
		/*
		// I Logic Here:
		// Increment or Decrement by PID_Params._I no value.
		// Reset I whenever sign of value changed because this means overshot.
		*/
		//if ((abs(Value - PID_Term->Error) > AbsValue ) || (Value ==0))
		//{  // Zero I if different signs.
			//PID_Term->I =0; ..... removed because it nakes whobbles.S
		//}
		//else 
		int16_t DeltaError = (Value - PID_Term->Error);
		if (Value > ACC_I_MIN)
		{
			PID_Term->I += (float)(PID_Params._I / 100.0f);						    		
		}
		else if (Value < -ACC_I_MIN)
		{	
			PID_Term->I -= (float)(PID_Params._I / 100.0f );						    		
		}
		else
		{
			PID_Term->I -= (float)(PID_Term->I * PID_I_LEAK_RATE);
		}			
				
		
		PID_Term->D= (float)((float)(DeltaError) * (float)PID_Params._D) / 20.0f ;
		PID_Term->Error = Value;	
		
				
		// Limit boundaries to custom values defined by user.
		PID_Term->I= Limiterf(PID_Term->I, PID_Params._ILimit);
		PID_Term->P= Limiterf(PID_Term->P, PID_Params._PLimit);
	    PID_Term->D= Limiterf(PID_Term->D, PID_Params._DLimit);
	
		Output = (PID_Term->P + PID_Term->I + PID_Term->D);	// P + I + D
		return  Output; 
}

float PID_Calculate (const pid_param_t PID_Params, pid_terms_t *PID_Term, const double  Value)
{
		float Output;
		
		// Calculate Terms 
	    PID_Term->P  = ((float)(Value * PID_Params._P) / 20.0f);						
		
		
		
		int16_t DeltaError = (Value - PID_Term->Error);
		
		/*
		// I Logic here:
		// DEAD band = 2
		// Increment or Decrement by Value * PID_Params._I 
		*/
		if ((Value > 1) || (Value < -1))
		{	// only increment I when the Value is increasing compared to the old one, also use [-2,2] as deadband.
			PID_Term->I += (float)((float)(Value * PID_Params._I) / 200.0f) ;	// try to replace Value with DeltaError
		}
		//else
		//{
			//PID_Term->I -= (float)(PID_Term->I * PID_I_LEAK_RATE);
		//}	
		
		
		
		PID_Term->D= (float)(DeltaError * PID_Params._D) / 20.0f ;
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