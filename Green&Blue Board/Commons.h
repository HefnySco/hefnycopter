/*
 * Commons.h
 *
 * Created: 23-Jul-12 3:50:06 PM
 *  Author: M.Hefny
 */ 


#ifndef COMMONS_H_
#define COMMONS_H_




/* DEFINITIONS */

// POD Values without any modifications.
#define MAX_POT_Extreme	1010
#define MAX_POT			1000
#define MID_POT			580
#define MIN_POT			100
#define MIN_POT_Extreme	50

// This one determines the range of the gain pot meters.
//Reducing the value by one doubles the range.
//Increasing the value by one halves the range.

#define PotScaleRoll	 10 // 1024   //2^10
#define PotScalePitch	 10 // 1024   //2^10
#define PotScaleYaw	     10 // 1024   //2^10


// This one determines the stick sensitivity.
// Reducing the value by one doubles the sensitivity.
// Increasing the value by one halves the sensitivity.

#define StickDivFactor	4	// 2^(StickDivFactor)

// This one determines the maximum Yaw command applied, in percent.
// Less gives less yaw authority, but also less possibility of motor saturation.
// More gives more yaw authority, but also more possibility of motor saturation during full rudder stick.

#define YawLimit	 30

#define LED_XMODE_TIMES		8
#define LED_PLUSMODE_TIMES	4
		
// Stick Arming - enable this line to enable Stick arming
#define STICK_LEFT  -10 //-100
#define STICK_RIGHT 10 //100
#define STICKThrottle_ARMING 10
#define STICKPOSITION_MIN	 16  // minimum time duration for stick to accept a command.
// Max Collective
// limits the maximum stick collective (range 80->100  100=Off)
// this allows gyros to stabilize better when full throttle applied
#define MAX_COLLECTIVE 180			

// trimming value for motors when generating PWM signals.
#define MOTORS_HIGH_VALUE	200  

// defines output rate to ESC/Servo
// either define by setting ESC_RATE (Max is approx 495Hz)
// uses Timer 1 ticks to control output rate.
//#define ESC_RATE 310	// in Hz
#define ESC_RATE 400	// in Hz (at SINGLE_COPTER and DUAL_COPTER)
//#define ESC_RATE 450	// in Hz
//#define ESC_RATE 495	// in Hz

//#define PWM_LOW_PULSE_INTERVAL  (((F_CPU / 8) / ESC_RATE) - 2000)/10
//#define PWM_LOW_PULSE_INTERVAL  ((1000000 / ESC_RATE) - 2000)/10
#define PWM_LOW_PULSE_INTERVAL (1000000 / ESC_RATE ) // 2020
//or define by setting PWM_LOW_PULSE_INTERVAL (minimum is 1)

#define BASE_PULSE  1104 / 16  //1120 / 16 :: I reduced it from 1120 to 1104 as trimming.

/* EOF - DEFINITIONS */


// TIMERS
volatile uint16_t TCNT1_X;				// This value overflow every 4294.967296 sec [1.19 hr], and tick every 0.065536 sec
volatile uint16_t OCR0A_X;

// GYRO
bool		GyroCalibrated;
volatile int16_t		gyroADC[3];						// Holds Gyro ADCs
volatile int16_t		gyroADC_updated[3];
volatile int16_t		gyroZero[3];					// Used for calibrating Gyros on ground

// POTS
volatile int16_t		GainInADC[3];
//volatile int8_t			GainIn[3];
volatile uint16_t		vBat;


/* Enumerations */

enum GyroDirection	{	GYRO_NORMAL = 0, GYRO_REVERSED };
enum GyroArrayIndex {	ROLL = 0, PITCH, YAW, ALT };
enum ADCInputs		{	YAW_GYRO = 0, PITCH_GYRO, ROLL_GYRO, ROLL_POT, PITCH_POT, YAW_POT, VBAT };

/* EOF - Enumerations */

#define LED_LONG_TOGGLE 100
#define LED_SHORT_TOGGLE 50

volatile bool Armed;


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

CONFIG_STRUCT Config;				// Holds configuration (from eeProm)





bool GyroCalibrated;


volatile bool RxChannelsUpdatedFlag;

volatile uint16_t RxChannel1;		// ISR vars: Signal duration . . . . valid as long as RxChannelsUpdatingFlag = 0
volatile uint16_t RxChannel2;
volatile uint16_t RxChannel3;
volatile uint16_t RxChannel4;

volatile int8_t RxInRoll;			// program vars: Normalized values of RXChanneln adjusted in RxGetChannels()
volatile int8_t RxInPitch;
volatile int8_t RxInCollective;
volatile int8_t RxInYaw;

volatile BOOL RxChannelsUpdatingFlag;


#endif /* COMMONS_H_ */