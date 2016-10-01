/*
 * Gyro_Sensor.h
 *
 * Created: 8/14/2012 2:37:28 AM
 *  Author: hefny
 */ 


#ifndef SENSOR_H_
#define SENSOR_H_

#if defined (KK21)
#define BAT_VOLT_RATIO	100/400
#else
#define BAT_VOLT_RATIO	100/380
#endif

#include "ADC_PORT.h"

		void Sensors_Init(void);
		char *Sensors_Test(uint8_t channel, uint16_t LowLimit, uint16_t HighLimit);
		//char * Sensor_GetBatteryTest(void);
		void Sensors_Calibrate (void);
		int16_t  Sensor_GetBattery(void);
		void Sensors_ReadAll (void);
		//int Sensors_GetAccAngle(int8_t Acc_Index);
		//int16_t Sensors_GetGyroRate(int8_t Gyro_Index);
		//void DynamicCalibration (void);
#endif /* SENSOR_H_ */