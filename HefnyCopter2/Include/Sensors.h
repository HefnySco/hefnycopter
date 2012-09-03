/*
 * Gyro_Sensor.h
 *
 * Created: 8/14/2012 2:37:28 AM
 *  Author: hefny
 */ 


#ifndef SENSOR_H_
#define SENSOR_H_

#include "ADC_PORT.h"

		void Sensors_Init(void);
		char *Sensors_Gyro_Test(uint8_t channel);
		char *Sensors_Acc_Test(uint8_t channel);
		void Sensors_Calibrate (void);
		uint16_t  Sensor_GetBattery(void);

#endif /* SENSOR_H_ */