/*
 * Acc_Sensor.cpp
 *
 * Created: 8/14/2012 2:26:30 AM
 *  Author: hefny
 */ 


#include <avr/io.h> 


#include "../Include/typedefs.h"
#include "../Include/Acc_Sensor.h"
#include "../Include/Misc.h"
#include "../Include/IO_config.h"


Acc_Sensor::Acc_Sensor()
{
	
}


void Acc_Sensor::Init(void)
{
	ACC_X  = INPUT;
	ACC_Y  = INPUT;
	ACC_Z  = INPUT;
}