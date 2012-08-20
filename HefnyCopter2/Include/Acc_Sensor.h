/*
 * Acc_Sensor.h
 *
 * Created: 8/14/2012 2:26:41 AM
 *  Author: hefny
 */ 


#ifndef ACC_SENSOR_H_
#define ACC_SENSOR_H_

#include "ADC_PORT.h"


	
	void Acc_Init(void);
	char *Acc_Test(uint8_t channel);


#endif /* ACC_SENSOR_H_ */