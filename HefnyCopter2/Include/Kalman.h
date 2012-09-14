/*
 * Kalman.h
 *
 * Created: 14-Sep-12 4:33:34 AM
 *  Author: M.Hefny
 */ 


#ifndef KALMAN_H_
#define KALMAN_H_



  float kalmanCalculate(int8_t index, float newAngle, float newRate,uint16_t looptime);
   


#endif /* KALMAN_H_ */