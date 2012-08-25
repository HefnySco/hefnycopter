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
#define LED_LONG_TOGGLE 800
#define LED_SHORT_TOGGLE 400

// LCD
#define LCD_RefreashRate	10

// KeyBoard
uint8_t _mykey;
uint8_t _TXKeys;
#define KEY_INIT	1
#define ISINIT		(_mykey & KEY_INIT)
#define KEY1		(_mykey & KEY_1)
#define KEY2		(_mykey & KEY_2)
#define KEY3		(_mykey & KEY_3)
#define KEY4		(_mykey & KEY_4)  
#define ANYKEY		(_mykey)
#define KEYPRESS	(_mykey & (KEY_1|KEY_2|KEY_3|KEY_4))
#define NOKEYRETURN {if (!_mykey) return;}



// Stick Arming - enable this line to enable Stick arming
#define STICK_LEFT				 400 //  [-500, 500]
#define STICK_RIGHT				-400 //
#define STICKThrottle_ARMING		50
#define STICKPOSITION_LONG_TIME		305  // minimum time duration for stick to accept a command.
#define STICKPOSITION_SHORT_TIME	80
// Max Collective
// limits the maximum stick collective (range 80->100  100=Off)
// this allows gyros to stabilize better when full throttle applied
#define MAX_COLLECTIVE 180
// trimming value for motors when generating PWM signals.
#define MOTORS_HIGH_VALUE	200  

// Defines output rate to ESC/Servo
// either define by setting ESC_RATE (Max is approx 495Hz)
// uses Timer 1 ticks to control output rate.
//#define ESC_RATE 310	// in Hz
#define ESC_RATE 400	// in Hz (at SINGLE_COPTER and DUAL_COPTER)
//#define ESC_RATE 450	// in Hz
//#define ESC_RATE 495	// in Hz
#define PWM_LOW_PULSE_INTERVAL (20000000 / ESC_RATE ) // 2020
#define BASE_PULSE  22080 / 16  //1120 / 16 :: I reduced it from 1120 to 1104 as trimming.
#define PWM_LOW_PULSE_INTERVAL (20000000 / ESC_RATE ) // 2020




////////////////////////////// Variables Section

// Receiver Signal Values

volatile uint16_t RX        [RXChannels];
volatile int16_t  RX_Latest [RXChannels];
// used for calibration...not initialized... true values are in Config in case IsCalibrated & Stick = True.
uint16_t RX_MAX_raw			[RXChannels];
uint16_t RX_MIN_raw			[RXChannels];

BOOL	IsArmed;

// Motors Signals
uint16_t MotorOut1;
uint16_t MotorOut2;
uint16_t MotorOut3;
uint16_t MotorOut4;




// TIMERS
volatile uint16_t TCNT1_X;				// This value overflow every 4294.967296 sec [1.19 hr], and tick every 0.065536 sec
volatile uint16_t TCNT2_X;				// This value overflow every 4294.967296 sec [1.19 hr], and tick every 0.065536 sec
//volatile uint16_t OCR0A_X;
uint16_t TCNT1_X_snapshot1;
uint16_t TCNT2_X_snapshot2;
BOOL bResetTCNR1_X;

// ADC Values

volatile uint16_t ADCValues[8];

volatile char Result[8]; 
volatile uint16_t nResult[8];

#define CALIBRATED_ALL		3
#define CALIBRATED_Stick	1
#define CALIBRATED_SENSOR	2

// eeProm data structure

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
	pid_param_t PID_SelfLevel;
	uint8_t AccTrimPitch;
	uint8_t AccTrimRoll;
	//model_t Mixer;
} config_t;

config_t Config;
#endif /* GLOBALVALUES_H_ */