/*
 * PID.h
 *
 * Created: 9/14/2012 6:25:21 AM
 *  Author: hefny
 */ 


#ifndef PID_H_
#define PID_H_


//int16_t P2D_Calculate (pid_param_t PID_Params, pid_terms_t PID_Term, int16_t  Gyro_Value, int16_t HumanStick, int16_t CompAngle);
//int16_t PID_Calculate (pid_param_t PID_GyroParams, pid_terms_t PID_Term, int16_t  Gyro_Value, pid_param_t PID_AccParams, int16_t CompAngle);

//float  PID_Calculate_ACC (const pid_param_t PID_Params, pid_terms_t *PID_Term, const double  Value);
float  PID_Calculate     (const pid_param_t PID_Params, pid_terms_t *PID_Term, const double  Value, const int8_t PIDFactorSelector);
void ZERO_Is();

#endif /* PID_H_ */