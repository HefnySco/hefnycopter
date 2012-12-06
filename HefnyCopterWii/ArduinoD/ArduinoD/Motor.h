/*
 * Motor.h
 *
 * Created: 04-Dec-12 3:20:23 AM
 *  Author: M.Hefny
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_

#include "typedef.h"
#include "Common.h"

	void DisarmMotors();
	void ArmMotors();

	void GetStabilizationOutput(structPhysicalMotor *PhysicalMotor, structVirtualMotor *VirtualMotors);	

	
	void GetNavigationOutput(structPhysicalMotor *PhysicalMotor,structRXRemote *RXRemote);
	

#endif /* MOTOR_H_ */


