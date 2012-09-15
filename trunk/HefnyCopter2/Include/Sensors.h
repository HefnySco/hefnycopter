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
		char *Sensors_Test(uint8_t channel, uint16_t LowLimit, uint16_t HighLimit);
		char * Sensor_GetBatteryTest(void);
		void Sensors_Calibrate (void);
		uint16_t  Sensor_GetBattery(void);
		void Sensors_ReadAll (void);
		int Sensors_GetAccAngle(int8_t Acc_Index);
		int16_t Sensors_GetGyroRate(int8_t Gyro_Index);

#endif /* SENSOR_H_ */