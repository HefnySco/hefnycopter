/*
 * Math.h
 *
 * Created: 14-Sep-12 4:42:54 AM
 *  Author: M.Hefny
 */ 


#ifndef MATH_H_
#define MATH_H_

int arctan2(int y, int x) ;

int16_t Limiter (int16_t Value, int16_t Limit);

int16_t ScaleSensor (int16_t SensorValue, int16_t minSource, int16_t maxSource, int16_t minDest, int16_t maxDest, double Ratio);

#endif /* MATH_H_ */