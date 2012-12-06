/*
 * typedef.h
 *
 * Created: 04-Dec-12 12:49:39 AM
 *  Author: M.Hefny
 */ 


#ifndef TYPEDEF_H_
#define TYPEDEF_H_
#include "sam.h"
#include <fastmath.h>

#include "config.h"

#define SIGNATURE			0x87
#define MINOR_VERSION		01
#define MAJOR_VERSION		00


////////////////////// Motors
#define MIN_MOTOR_VALUE	10
#define MAX_MOTOR_VALUE	900



/////////////////////////Motor.h

typedef struct 
{
	int32_t Thrust;
	int32_t Pitch_Balance_Output;
	int32_t Roll_Balance_Output;
	int32_t YAW_Output;
} structVirtualMotor;


typedef struct 
{
	/*
	// Angle between multi-rotor front and physical arm.
	// Plus[Front:0.00, Right:-PI/2, Back:PI, Left:PI/2]
	// Plus-X-Copter ...etc.
	*/
	double CitaAngle; 	
	/*
	// 1 & -1 used based on propeller to use it in YAW
	*/
	int8_t MotorDirection;
	/*
	//	Fixed trim.
	*/
	int32_t	Trim;
	
	int32_t MotorOutput;	
} structPhysicalMotor;

#define MOTOR_CW		1
#define MOTOR_CCW	   -1
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////

#define REMOTE_NOTCALIBRATED				0
#define PRIMARY_REMOTE_CALIBRATED			1
#define SECONDARY_REMOTE_CALIBRATED			2
#define SET_PRIMARY_REMOTE_CALIBRATED		(x)	(x | 0x00000001)
#define SET_SECONDARY_REMOTE_CALIBRATED		(x)	(x | 0x00000010)
#define IS_PRIMARY_REMOTE_CALIBRATED		(x)	(x & 0x00000001)
#define IS_SECONDARY_REMOTE_CALIBRATED		(x)	(x & 0x00000010)


#define VIRTUAL_DIRECTIONS	4 // [Front, Right, back, Left]
#define MAX_MOTOR_COUNT		8
/////////////////////////RX.h


#define PRIMARY_REMOTE		1
#define SECONDARY_REMOTE	2

#define REMOTECHANNELS		9
#define THROTTLECHANNEL		0
#define AILERONCHANNEL		1
#define ELEVATORCHANNEL		2
#define RUDDERCHANNEL		3
#define GEARCHANNEL			4
#define AUX1CHANNEL			5
#define AUX2CHANNEL			6
#define AUX3CHANNEL			7


typedef struct
{
	int32_t CurrentValue;
	int32_t MinValue;
	int32_t MaxValue;
	int32_t ZeroValue;		// This is the middle value for Rud, Ele, Ail.
} structRXChannel;

typedef struct
{
	structRXChannel RXChannels[REMOTECHANNELS];
	
} structRXRemote;



//////////////////////////////////////////////////////////////////////
typedef struct
{
	// Signature & Version
	char					Signature;
	int8_t					MinorVerson;
	int8_t					MajorVersion;
	// ESC Calibration
	int8_t					ESC_Calibration;
	int8_t					SENSORS_Calibrated;
	int8_t					RX_Calibrated;
	// Motors
	int16_t					MinThrottle;
	int16_t					MaxThrottle;
	int8_t					NumberOfMotors;
	structPhysicalMotor		PhysicalMotor[MAX_MOTOR_COUNT];		
} structData;


#endif /* TYPEDEF_H_ */


