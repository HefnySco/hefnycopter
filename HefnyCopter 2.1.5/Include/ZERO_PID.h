/*
 * ZERO_PID.h
 *
 * Created: 03-Jun-14 12:34:32 PM
 *  Author: M.Hefny
 */ 


#ifndef ZERO_PID_H_
#define ZERO_PID_H_


#ifdef G_TUNE
void Init_ZEROPID ();
void Calculate_ZEROPID (uint8_t Axis, double Error);

#endif


#endif /* ZERO_PID_H_ */