/*
 * PID.h
 *
 * Created: 9/14/2012 6:25:21 AM
 *  Author: hefny
 */ 


#ifndef PID_H_
#define PID_H_



int16_t P2D_Calculate (pid_terms_t PID_Term, double  Gyro_Value, int16_t HumanStick, double CompAngle);

#endif /* PID_H_ */