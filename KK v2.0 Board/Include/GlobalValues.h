/*
 * GlobalValues.h
 *
 * Created: 8/16/2012 7:20:55 AM
 *  Author: hefny
 */ 


#ifndef GLOBALVALUES_H_
#define GLOBALVALUES_H_

///////////////////////////// Define Section

// LED
#define LED_LONG_TOGGLE 100
#define LED_SHORT_TOGGLE 50

// Stick Arming - enable this line to enable Stick arming
#define STICK_LEFT  -20 //-100
#define STICK_RIGHT  20 //100
#define STICKThrottle_ARMING 10
#define STICKPOSITION_MIN	 16  // minimum time duration for stick to accept a command.
// Max Collective
// limits the maximum stick collective (range 80->100  100=Off)
// this allows gyros to stabilize better when full throttle applied
#define MAX_COLLECTIVE 180



////////////////////////////// Variables Section

// Receiver Signal Values
uint16_t RxInCollective;
uint16_t RxInYaw;
uint16_t RxInRoll;
uint16_t RxInPitch;

bool	bArmed;

// Motors Signals
uint16_t MotorOut1;
uint16_t MotorOut2;
uint16_t MotorOut3;
uint16_t MotorOut4;





// TIMERS
volatile uint16_t TCNT1_X;				// This value overflow every 4294.967296 sec [1.19 hr], and tick every 0.065536 sec
volatile uint16_t OCR0A_X;


#endif /* GLOBALVALUES_H_ */