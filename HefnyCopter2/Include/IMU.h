/*
 * IMU.h
 *
 * Created: 30-Aug-12 8:51:40 AM
 *  Author: M.Hefny
 */ 


#ifndef IMU_H_
#define IMU_H_


// IMU Calculations

#define RAD_TO_DEG 57.295779513082320876798154814105

float CompAngleX;
float CompAngleY;

void IMU_CalculateAngles ();
double 	y;
			
int16_t ScaleSensor (int16_t SensorValue, pid_param_t *pid_Param, double Ration);
void IMU_PID (void);
#endif /* IMU_H_ */