/*
 * MathHelper.c
 *
 * Created: 04-Dec-12 9:40:55 AM
 *  Author: M.Hefny
 */ 


#include "MathHelper.h"


int32_t Constraint (int32_t Value, int32_t MinValue, int32_t MaxValue)
{
	if (Value <= MinValue) return MinValue;
	if (Value >= MaxValue) return MaxValue;
}