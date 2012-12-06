/*
 * Common.h
 *
 * Created: 04-Dec-12 9:12:13 AM
 *  Author: M.Hefny
 */ 


#ifndef COMMON_H_
#define COMMON_H_


#include "typedef.h"


static structData	EEPROMData;

// Output Driver
static double	HeadingAngler;
static structVirtualMotor VirtualMotors;
// RX Data
static structRXRemote	RXRemote[2]; // support two Remotes.

//////////////////// Motor.h
bool			Armed;
structPhysicalMotor PhysicalMotor[MAX_MOTOR_COUNT];

#endif /* COMMON_H_ */


