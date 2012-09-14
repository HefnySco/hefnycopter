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
float CompAngleZ;



void IMU_Kalman (void);
void IMU_CalculateAngles (void);
void IMU_P2D (void);
#endif /* IMU_H_ */