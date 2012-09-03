/*
 * GlobalValues.h
 *
 * Created: 8/16/2012 7:20:55 AM
 *  Author: hefny
 */ 


#ifndef GLOBALVALUES_H_
#define GLOBALVALUES_H_



#include "typedefs.h"
#include "IO_config.h"
///////////////////////////// Define Section

//Timer
//#define 

// LED
#define LED_LONG_TOGGLE  80  // 800 
#define LED_SHORT_TOGGLE 30 // 400

// LCD
#define LCD_RefreashRate	10
char sXDeg[10];

 


// Stick Arming - enable this line to enable Stick arming
#define STICK_LEFT				 400	//  the total range is from [-500, 500]
#define STICK_RIGHT				-400	
#define STICKThrottle_HIGH			850	//  the total range is from [ 0, 1000]
#define STICKThrottle_ARMING		50	//  the total range is from [ 0, 1000]
#define STICKPOSITION_LONG_TIME		305  // minimum time duration for stick to accept a command.
#define STICKPOSITION_SHORT_TIME	80
// Max Collective
// limits the maximum stick collective (range 80->100  100=Off)
// this allows gyros to stabilize better when full throttle applied
#define MAX_COLLECTIVE		180
// trimming value for motors when generating PWM signals.
#define MOTORS_HIGH_VALUE	1000  
#define MOTORS_IDLE_VALUE	40
// Defines output rate to ESC/Servo
// either define by setting ESC_RATE (Max is approx 495Hz)
// uses Timer 1 ticks to control output rate.
#define ESC_RATE 50	// in Hz
//#define ESC_RATE 400	// in Hz (at SINGLE_COPTER and DUAL_COPTER)
//#define ESC_RATE 450	// in Hz
//#define ESC_RATE 495	// in Hz
#define PWM_LOW_PULSE_INTERVAL 1000000 / ESC_RATE // time in ms for a delay.
#define BASE_PULSE				1000   //  :: I reduced it from 1120 to 1104 as trimming.





////////////////////////////// Variables Section

// Receiver Signal Values

volatile uint16_t RX        [RXChannels];
volatile int16_t  RX_Latest [RXChannels];   // the actual RX values that are used for calculations.
// used for calibration...not initialized... true values are in Config in case IsCalibrated & Stick = True.
uint16_t RX_MAX_raw			[RXChannels];
uint16_t RX_MIN_raw			[RXChannels];

volatile BOOL RX_Good;

BOOL	IsArmed;

// Motors Signals
uint16_t MotorOut1;
uint16_t MotorOut2;
uint16_t MotorOut3;
uint16_t MotorOut4;




// TIMERS
volatile uint16_t TCNT1_X;				// TCNT1_X click every 0.0032768 sec [1 sec = 305.17578125 TCNT1_X]
volatile uint16_t TCNT2_X;				// TCNT2  overflows every  3.2us x 0xff = 0.0008192 sec,  TCNT2_X value tick every 819.2 us and overflow every 53.6870912 sec
//volatile uint16_t OCR0A_X;
uint16_t TCNT1_X_snapshot1;
uint16_t TCNT_X_snapshot2;
BOOL bResetTCNR1_X;



// ADC Values
volatile uint16_t ADCValues[8];
uint16_t Sensors_Latest [8];
volatile char Result[8]; 
volatile uint16_t nResult[8];
uint16_t nTemp16;
int16_t  iTemp16;

#define CALIBRATED_ALL		3
#define CALIBRATED_Stick	1
#define CALIBRATED_SENSOR	2

// eeProm data structure

#define IMU_SelfLevelMode	1
typedef struct  
{
	uint8_t signature;					
	uint8_t IsCalibrated;
	uint8_t RX_mode;
	uint16_t RX_Mid[RXChannels];
	uint16_t RX_Min[RXChannels];
	//uint8_t RX_chmap[RXChannels];
	uint16_t Sensor_zero[SENSORS_ALL];
	pid_param_t PID[3];
	uint8_t SelfLevelMode;
	uint8_t ArmingMode;
	uint8_t AutoDisarm;
	uint8_t LinkRollPitch;
	uint8_t ReceiverMode;
	uint8_t MixerIndex;
	uint8_t StickScaling[4];
	uint8_t MinThrottle;
	uint8_t LCDContrast;
	uint8_t HeightDampening;
	uint8_t HeightDampeningLimit;
	uint8_t LVA;
	//pid_param_t PID_SelfLevel;
	uint8_t	AccGain;
	uint8_t AccTrim;
	//model_t Mixer;
} config_t;

config_t Config;




#endif /* GLOBALVALUES_H_ */