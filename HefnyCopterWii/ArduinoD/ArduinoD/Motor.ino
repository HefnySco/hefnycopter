/*
 * Motor.cpp
 *
 * Created: 04-Dec-12 3:19:57 AM
 *  Author: M.Hefny
 */ 
 
#include <MathHelper.h>
#include "Motor.h"



 void DisarmMotors()
 {
	Armed = false;
	// LED OFF
	// BEEP OFF
 }
 
 void ArmMotors()
 {
	Armed = true;
	// LED ON
	// BEEP ON
 }

void GetStabilizationOutput(structPhysicalMotor *PhysicalMotor, structVirtualMotor *VirtualMotors)
{
	
	int32_t Output;
	for (int j,i=0; i<EEPROMData.NumberOfMotors; ++i)
	{
		if (Armed == false)
		{
			PhysicalMotor[i].MotorOutput = 0;
		}
		else
		{
			Output = VirtualMotors[j].Thrust
					+ 
					(
						PhysicalMotor[i].MotorDirection
					  * VirtualMotors->YAW_Output
					)
					+
					(
						cos(PhysicalMotor[i].CitaAngle) 
					  * VirtualMotors->Pitch_Balance_Output 
					)
					+
					(
						cos(PhysicalMotor[i].CitaAngle - PI_2) 
					  * VirtualMotors->Roll_Balance_Output 
					);
			
			PhysicalMotor[i].MotorOutput = Output;// Constraint(Output, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE);		 

		}
		
		#ifdef DEBUG_SERIAL
		   Serial.println("Sta: Motor:" + String (i));
		   Serial.println("Sta: Output:" + String (PhysicalMotor[i].MotorOutput));
		#endif	
	}	
}


void GetNavigationOutput(structPhysicalMotor *PhysicalMotor,structRXRemote *RXRemote)
{
	
	int32_t Output;
	
	for (int i=0; i<EEPROMData.NumberOfMotors; ++i)
	{
		if (Armed == false)
		{
			PhysicalMotor[i].MotorOutput = 0;
		}
		else
		{
			Output = (
				cos(HeadingAngler + PhysicalMotor[i].CitaAngle )
					* RXRemote->RXChannels[ELEVATORCHANNEL].CurrentValue
					)
					+
					(
					cos(HeadingAngler + PhysicalMotor[i].CitaAngle - PI_2)
					* RXRemote->RXChannels[AILERONCHANNEL].CurrentValue
					)
					+ RXRemote->RXChannels[THROTTLECHANNEL].CurrentValue
					+
					(
						RXRemote->RXChannels[RUDDERCHANNEL].CurrentValue
						* PhysicalMotor[i].MotorDirection
					)
					+ PhysicalMotor[i].Trim;
					
					PhysicalMotor[i].MotorOutput = Output; //Constraint(Output, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE);
		}					

		#ifdef DEBUG_SERIAL
		   Serial.println("Nav: Motor: " + String(i));
		   Serial.println("Nav: Output:" + String(PhysicalMotor[i].MotorOutput));
		#endif	
	}
}


