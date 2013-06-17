/*
 * IMU.h
 *
 * Created: 30-Aug-12 8:51:40 AM
 *  Author: M.Hefny
 */ 


#ifndef IMU_H_
#define IMU_H_


// IMU Calculations


//double CompAngleX;
//double CompAngleY;
//double  CompAngleZ;



//void IMU_Kalman (void);
//void IMU_CalculateAngles (void);
void IMU_Reset();
void IMU (void);
double IMU_HeightKeeping ();
#endif /* IMU_H_ */