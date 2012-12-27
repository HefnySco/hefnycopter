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



/*
int16_t P2D_Calculate (pid_param_t PID_Params, pid_terms_t PID_Term, int16_t  Gyro_Value, int16_t HumanStick, int16_t CompAngle)
{
		int16_t Output;
		
		if ((CompAngle>=-2) && (CompAngle<=2))
		CompAngle=0;
		
		
		// PITCH
		if ((Gyro_Value>=-3) && (Gyro_Value<=3)) 
		{
			////PID_Term->P = 0;
			////PID_Term->I = 0; //-(CompAngle * PID_Params._I) / 10;						    
			////PID_Term->D = 0;
			
			return 0;
		}
		else
		{
			// Calculate Terms 
			PID_Term->P = (Gyro_Value  * PID_Params._P) / 10;						
			PID_Term->I = (CompAngle * PID_Params._I) / 10;						    
			PID_Term->D= ((Gyro_Value - PID_Term->Error) * PID_Params._D) / 10;
			PID_Term->Error = Gyro_Value;	
		}
				
		// Limit boundaries to custom values defined by user.
		PID_Term->I= Limiter(PID_Term->I, PID_Params._ILimit);
		PID_Term->P= Limiter(PID_Term->P, PID_Params._PLimit);
	    PID_Term->D= Limiter(PID_Term->D, PID_Params._DLimit);
	
		Output = (PID_Term->P + PID_Term->I + PID_Term->D);	// P + I + D
		return  Limiter(Output,(int16_t)300);
}		
*/

//
//int16_t PID_Calculate (pid_param_t PID_GyroParams, pid_terms_t PID_Term, int16_t  Gyro_Value, pid_param_t PID_AccParams, int16_t CompAngle)
//{
		//int16_t Output;
		//
		////if ((CompAngle>=-2) && (CompAngle<=2))
		////CompAngle=0;
		//
		//
	    //// Calculate Terms 
	    //PID_Term->P  = (Gyro_Value * PID_GyroParams._P);						
		//PID_Term->P += (CompAngle * PID_AccParams._P);						
		//PID_Term->P = PID_Term->P / 10;
		//
		//PID_Term->I += ((Gyro_Value * PID_GyroParams._I) / 10);						    
		//PID_Term->I += ((CompAngle * PID_AccParams._I) / 10);						    
		//
		//PID_Term->D= ((Gyro_Value - PID_Term->Error) * PID_GyroParams._D) / 10;
		////PID_Term->D+= ((CompAngle - PID_AccParams.Error) * PID_Params._D) / 10;
		//PID_Term->Error = Gyro_Value;	
		//
				//
		//// Limit boundaries to custom values defined by user.
		//PID_Term->I= Limiter(PID_Term->I, PID_GyroParams._ILimit);
		//PID_Term->P= Limiter(PID_Term->P, PID_GyroParams._PLimit);
	    //PID_Term->D= Limiter(PID_Term->D, PID_GyroParams._DLimit);
	//
		//Output = (PID_Term->P + PID_Term->I + PID_Term->D);	// P + I + D
		//
		//return  Output; //Limiter(Output,(int16_t)300);
//}		
//


float PID_Calculate_ACC (pid_param_t PID_Params, pid_terms_t *PID_Term, double  Value)
{
	float Output;
		
		// Calculate Terms 
	    PID_Term->P  = ((float)(Value * PID_Params._P) / 5.0f);						
		
		// Increment by 1 always ... dont use value to increment.
		
		
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
		
		if (Value > 0)
		{
			PID_Term->I += (float)(PID_Params._I / 10.0f);						    		
		}
		else if (Value < 0)
		{	
			PID_Term->I -= (float)(PID_Params._I / 10.0f );						    		
		}
		//else
		//{
			//PID_Term->I -= (float)(PID_Term->I * PID_I_LEAK_RATE);
		//}			
				
		
		PID_Term->D= (float)((float)(DeltaError) * (float)PID_Params._D) / 5.0f ;
		PID_Term->Error = Value;	
		
				
		// Limit boundaries to custom values defined by user.
		PID_Term->I= Limiterf(PID_Term->I, PID_Params._ILimit);
		PID_Term->P= Limiterf(PID_Term->P, PID_Params._PLimit);
	    PID_Term->D= Limiterf(PID_Term->D, PID_Params._DLimit);
	
		Output = (PID_Term->P + PID_Term->I + PID_Term->D);	// P + I + D
		//Output = Output / 10;
		return  Output; //Limiter(Output,(int16_t)300);
	
}

float PID_Calculate (pid_param_t PID_Params, pid_terms_t *PID_Term, double  Value)
{
		float Output;
		
		// Calculate Terms 
	    PID_Term->P  = ((float)(Value * PID_Params._P) / 10.0f);						
		
		
		
		int16_t DeltaError = (Value - PID_Term->Error);
		
		/*
		// I Logic here:
		// DEAD band = 2
		// Increment or Decrement by Value * PID_Params._I 
		*/
		if ((Value > 2) || (Value < -2))
		{	// only increment I when the Value is increasing compared to the old one, also use [-2,2] as deadband.
			PID_Term->I += (float)((float)(Value * PID_Params._I) / 100.0f) ;	// try to replace Value with DeltaError
		}
		//else
		//{
			//PID_Term->I -= (float)(PID_Term->I * PID_I_LEAK_RATE);
		//}	
		
		
		
		PID_Term->D= (float)(DeltaError * PID_Params._D) / 10.0f ;
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
	PID_GyroTerms[0].I=0;
	PID_GyroTerms[1].I=0;
//	PID_GyroTerms[2].I=0;
	PID_AccTerms[0].I=0;
	PID_AccTerms[1].I=0;
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