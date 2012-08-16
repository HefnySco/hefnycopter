/*
 * Gyro_Sensor.cpp
 *
 * Created: 8/14/2012 2:37:11 AM
 *  Author: hefny
 */ 


#include <avr/io.h> 


#include "../Include/typedefs.h"
#include "../Include/Gyro_Sensor.h"
#include "../Include/Misc.h"
#include "../Include/IO_config.h"


Gyro_Sensor::Gyro_Sensor()
{
	
}


void Gyro_Sensor::Init(void)
{
	GYRO_X = INPUT;
	GYRO_Y = INPUT;
	GYRO_Z = INPUT;
}