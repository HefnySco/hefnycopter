/*
 * Kalman.h
 *
 * Created: 14-Sep-12 4:33:34 AM
 *  Author: M.Hefny
 */ 


#ifndef KALMAN_H_
#define KALMAN_H_



  int16_t Kalman_Calculate(int8_t index, int16_t newAngle, int16_t newRate,uint16_t looptime);
   


#endif /* KALMAN_H_ */