/*
 * Gyro_Sensor.h
 *
 * Created: 8/14/2012 2:37:28 AM
 *  Author: hefny
 */ 


#ifndef GYRO_SENSOR_H_
#define GYRO_SENSOR_H_

#include "ADC_PORT.h"

class Gyro_Sensor : public ADC_PORT
{
	public:
		Gyro_Sensor();
		void Init(void);
		
};


#endif /* GYRO_SENSOR_H_ */