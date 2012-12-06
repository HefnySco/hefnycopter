/*
 * PID.h
 *
 * Created: 05-Dec-12 2:25:53 PM
 *  Author: M.Hefny
 */ 


#ifndef PID_H_
#define PID_H_

#include "sam.h"
#include <fastmath.h>

////////////////////// PID Structure

typedef struct
{
	
	int32_t P_Param, P_Limit;
	int32_t I_Param, I_Limit;
	int32_t D_Param, D_Limit;
	
} structPIDParams;

typedef struct
{
	int32_t P_Value;
	int32_t I_Value;
	int32_t D_Value;
	int32_t Value_1, Value_2;
	
} structPIDValues;


class PID
{
	public:
		
};

#endif /* PID_H_ */