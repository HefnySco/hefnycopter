/*
 * Init.c
 *
 * Created: 05-Dec-12 8:58:43 PM
 *  Author: M.Hefny
 */ 

#include "Init.h"
#include "Common.h"

#include "Sensors.h"

void InitData ()
{
  
#ifdef DEBUG_SERIAL
        Serial.print("InitData\r\n");
#endif	

	EEPROMData.Signature			= SIGNATURE;
	EEPROMData.MinorVerson 			= MINOR_VERSION;
	EEPROMData.MajorVersion 		= MAJOR_VERSION;
	
	EEPROMData.ESC_Calibration		= 0;
	EEPROMData.RX_Calibrated		= REMOTE_NOTCALIBRATED;
	EEPROMData.SENSORS_Calibrated	= SENSORS_NOTCALIBRATED;
	
	EEPROMData.MinThrottle 			= MIN_MOTOR_VALUE;
	EEPROMData.MaxThrottle 			= MAX_MOTOR_VALUE;
	
	// Default is QUAD
	EEPROMData.NumberOfMotors = 4;	
	
	// Default is PLUS
	EEPROMData.PhysicalMotor[0].CitaAngle=0;
	EEPROMData.PhysicalMotor[0].Trim=0;
	EEPROMData.PhysicalMotor[0].MotorDirection=MOTOR_CW;
	EEPROMData.PhysicalMotor[0].MotorOutput=0;
	
	EEPROMData.PhysicalMotor[1].CitaAngle=PI_2;
	EEPROMData.PhysicalMotor[1].Trim=0;
	EEPROMData.PhysicalMotor[1].MotorDirection=MOTOR_CCW;
	EEPROMData.PhysicalMotor[1].MotorOutput=0;
	
	EEPROMData.PhysicalMotor[2].CitaAngle=PI;
	EEPROMData.PhysicalMotor[2].Trim=0;
	EEPROMData.PhysicalMotor[2].MotorDirection=MOTOR_CW;
	EEPROMData.PhysicalMotor[2].MotorOutput=0;
	
	EEPROMData.PhysicalMotor[3].CitaAngle=PI_3x2;
	EEPROMData.PhysicalMotor[3].Trim=0;
	EEPROMData.PhysicalMotor[3].MotorDirection=MOTOR_CCW;
	EEPROMData.PhysicalMotor[3].MotorOutput=0;
	
	// save remaining motors with ZERO config.
	memset(&EEPROMData.PhysicalMotor[4],0,sizeof(structPhysicalMotor) * (MAX_MOTOR_COUNT-4));
	
}

void InitBoard (void)
{
	InitData();

	memcpy(PhysicalMotor,&EEPROMData.PhysicalMotor[0],sizeof(structPhysicalMotor) * MAX_MOTOR_COUNT);
	
	ArmMotors();
	VirtualMotors.Thrust=50;
	VirtualMotors.Pitch_Balance_Output=11;
	VirtualMotors.Roll_Balance_Output=22;
	VirtualMotors.YAW_Output=70;
	
	
	RXRemote[1].RXChannels[ELEVATORCHANNEL].CurrentValue=1;
	RXRemote[1].RXChannels[AILERONCHANNEL].CurrentValue=2;
	RXRemote[1].RXChannels[RUDDERCHANNEL].CurrentValue=3;
	HeadingAngler = PI_4;
}

