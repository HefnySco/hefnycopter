/*
 * ZERO_PID.c
 *
 * Created: 03-Jun-14 12:32:01 PM
 *  Author: M.Hefny
 * Kindly refer to the AUTHOR in case you use the ZERO_PID
 * http://technicaladventure.blogspot.com/2014/06/zero-pids-tuner-for-multirotors.html
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>


#include "../Include/typedefs.h"
#include "../Include/GlobalValues.h"
#include "../Include/Math.h"

#define P_INDEX	0
#define I_INDEX	1
#define D_INDEX	2

#define TUNED_BAND	4	
#define MAX_TUNED_P	1000 
#define MAX_TUNED_I	100

static pid_param_t TestParams[3];		

static int16_t OldError [3][3];
static int16_t Error_1 [3];


void Init_ZEROPID ()
{
	for (int i=0;i<3;++i)
	{
		Config.GyroParams[i]._P=0;
		Config.GyroParams[i]._I=0;
		Config.GyroParams[i]._D=0;
		TestParams[i]._P=0;
		TestParams[i]._PLimit = Config.GyroParams[i]._PLimit;
		TestParams[i]._I=0;
		TestParams[i]._ILimit = Config.GyroParams[i]._ILimit;
		TestParams[i]._D=0;
		TestParams[i]._DLimit = Config.GyroParams[i]._DLimit;
	}
	
}

void Calculate_ZEROPID (uint8_t Axis, double Error)
{

	
	int16_t diff_P;
	int16_t diff_I;
	Error = Error / 2;
	
	//1- Test if Error is in the tune band.
	if (abs(Error) < Config.GyroParams[Axis]._DLimit)
	{ // assuming Stable Band
		OldError[P_INDEX][Axis]=Error;
		OldError[I_INDEX][Axis]=Error;
		return ;
	}
	
	//2- If Error is same sign as old error then we should be undershoot.
	if ((Error >= 0 && OldError[P_INDEX][Axis] >=0) || (Error <= 0 && OldError[P_INDEX][Axis] <=0))	
	{	// Same Sign
		diff_P = (int16_t)(abs((int16_t)Error) - abs(OldError[P_INDEX][Axis]));
		
		
		if (diff_P > Config.GyroParams[Axis]._DLimit)
		{ // 2.1 still loosing more.
			if (Config.TestParams[Axis]._P < MAX_TUNED_P)  {Config.TestParams[Axis]._P +=1;}
		}
		else if (diff_P < -Config.TestParams[Axis]._DLimit)
		{ // 2.2 we are catching up now.
			/////if ( Config.GyroParams[Axis]._P > 0)	{Config.GyroParams[Axis]._P  -=1;}
		}
	}else if (Error < 0)  // 3. different sign then we overshot Note: Error is already out of tune band
	{   // from - to + 
		if ( Config.TestParams[Axis]._P > 2)	{Config.TestParams[Axis]._P  -=2;} erlse Config.TestParams[Axis]._P =0;
	}	
	else if (Error > 0)	
	{  // from + to - 
		if ( Config.TestParams[Axis]._P > 2)	{Config.TestParams[Axis]._P  -=2;} else Config.TestParams[Axis]._P=0;
	}			
		
	if ((Error >= 0 && OldError[I_INDEX][Axis] >=0) || (Error <= 0 && OldError[I_INDEX][Axis] <=0))	
	{
		diff_I = (int16_t)(abs((int16_t)Error) - abs(OldError[I_INDEX][Axis]));
		
		if (diff_I > Config.TestParams[Axis]._DLimit)
		{
			if (Config.TestParams[Axis]._I < MAX_TUNED_I) Config.TestParams[Axis]._I +=1; 
		}
		else if (diff_I < -Config.TestParams[Axis]._DLimit)
		{
		/////	if ( Config.GyroParams[Axis]._I > 0)	Config.GyroParams[Axis]._I -=1; 
		}
	}
	else if (Error < 0)
	{   // from - to + 
		if ( Config.TestParams[Axis]._I > 2)	Config.TestParams[Axis]._I -=2; else Config.TestParams[Axis]._I =0;
	}	
	else if (Error > 0)	
	{  // from + to - 
		if ( Config.TestParams[Axis]._I > 2)	Config.TestParams[Axis]._I -=2; else Config.TestParams[Axis]._I =0;
	}	
				
	
	
	  
	// P & I for PITCH
	OldError[P_INDEX][Axis] = (int16_t)( 1 * OldError[P_INDEX][Axis] + (int16_t)Error) / 2;
	OldError[I_INDEX][Axis] = (int16_t)( 19 * OldError[I_INDEX][Axis] + (int16_t)Error) / 20;
	
	Error_1[Axis]= (int16_t)Error;
	
	Config.GyroParams[Axis]._P = TestParams[Axis]._P / 10;
	Config.GyroParams[Axis]._I = TestParams[Axis]._I / 10 ;
}