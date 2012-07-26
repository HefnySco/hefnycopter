/*
 * motors.h
 *
 * Created: 22-Jul-12 9:33:17 PM
 *  Author: M.Hefny
 */ 


#ifndef MOTORS_H_
#define MOTORS_H_

volatile uint16_t pmm_out_step;
volatile void *pMotorFunction;

volatile int16_t MotorOut1;		// Motor speed variables
volatile int16_t MotorOut2;
volatile int16_t MotorOut3;
volatile int16_t MotorOut4;
volatile int16_t MotorOut5;
volatile int16_t MotorOut6;

void output_motor_ppm(void);
void output_motor_ppm2(void);

#endif /* MOTORS_H_ */