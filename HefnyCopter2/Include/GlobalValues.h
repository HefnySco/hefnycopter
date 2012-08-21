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

volatile uint16_t RX[RXChannels];
uint16_t RX_MAX_raw[RXChannels];
uint16_t RX_MIN_raw[RXChannels];

BOOL	bArmed;

// Motors Signals
uint16_t MotorOut1;
uint16_t MotorOut2;
uint16_t MotorOut3;
uint16_t MotorOut4;





// TIMERS
volatile uint16_t TCNT1_X;				// This value overflow every 4294.967296 sec [1.19 hr], and tick every 0.065536 sec
volatile uint16_t OCR0A_X;


// ADC Values

volatile uint16_t ADCValues[8];

volatile char Result[8]; 
volatile uint16_t nResult[8];


// eeProm data structure
typedef struct Config_Struct CONFIG_STRUCT;
struct Config_Struct
{
	uint8_t	setup;					// byte to identify if already setup

	uint8_t RollGyroDirection;
	uint8_t PitchGyroDirection;
	uint8_t YawGyroDirection;

	// allows setting to zero
	uint16_t RxChannel1ZeroOffset;
	uint16_t RxChannel2ZeroOffset;
	uint16_t RxChannel3ZeroOffset;  // currently fixed
	uint16_t RxChannel4ZeroOffset;

};




typedef struct  
{
	uint8_t signature;
	uint8_t IsCalibrated;
	uint8_t RX_mode;
	uint16_t RX_Max[RXChannels];
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