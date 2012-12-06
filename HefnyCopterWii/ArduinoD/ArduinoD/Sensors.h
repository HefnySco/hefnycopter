/*
 * Sensors.h
 *
 * Created: 05-Dec-12 10:13:12 PM
 *  Author: M.Hefny
 */ 


#ifndef SENSORS_H_
#define SENSORS_H_


#define SENSORS_NOTCALIBRATED				0
#define SET_ACC_CALIBRATED					(x)	(x | 0x00000001)
#define IS_ACC_CALIBRATED					(x)	(x & 0x00000001)
#define SET_MAG_CALIBRATED					(x)	(x | 0x00000010)
#define IS_MAG_CALIBRATED					(x)	(x & 0x00000010)
#define SET_BAR_CALIBRATED					(x)	(x | 0x00000100)
#define IS_BAR_CALIBRATED					(x)	(x & 0x00000100)


#endif /* SENSORS_H_ */


