/*
 * Timer.c
 *
 * Created: 8/18/2012 6:56:45 AM
 *  Author: hefny
 */ 


#ifndef TIMER_H_
#define TIMER_H_
 

volatile uint16_t TCNT1_X;				// TCNT1_X click every 0.0032768 sec [1 sec = 305.17578125 TCNT1_X]
volatile uint16_t TCNT2_X;				// TCNT2  overflows every  3.2us x 0xff = 0.0008192 sec,  TCNT2_X value tick every 819.2 us and overflow every 53.6870912 sec



void Timer_Init(void);


#endif