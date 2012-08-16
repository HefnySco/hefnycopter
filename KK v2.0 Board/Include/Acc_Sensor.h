/*
 * Acc_Sensor.h
 *
 * Created: 8/14/2012 2:26:41 AM
 *  Author: hefny
 */ 


#ifndef ACC_SENSOR_H_
#define ACC_SENSOR_H_

#include "ADC_PORT.h"

class Acc_Sensor : public ADC_PORT
{
	public:
		Acc_Sensor();
		void Init(void);
};


#endif /* ACC_SENSOR_H_ */