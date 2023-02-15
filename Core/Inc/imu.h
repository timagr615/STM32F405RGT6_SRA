/*
 * imu.h
 *
 *  Created on: Feb 3, 2023
 *      Author: tima
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "lsm6ds3.h"
#include "lis3mdl.h"

#define MAGNETIC_DECLINATION 10.91 // Moscow, 2022
#define ACC_LPF 0.4
#define GYRO_LPF 0.4
#define ROLL_FILTER 0.8
#define YAW_FILTER 0.8
#define PITCH_FILTER 0.8

#define DEG_TO_RAD M_PI / 180.0

typedef struct {
	// IMU Data
	float ap[3];
	float a[3];
	float gp[3];
	float g[3];
	float m[3];

	float aned[3];
	float gned[3];
	float mned[3];

	float q[4];  // vector to hold quaternion
	float rpyp[3];
	float rpy[3];
	float lin_acc[3];
	float earth_acc[3];
	float earth_gyro[3];
	float long_acc;
	uint8_t n_filter_iter;
	float magnetic_declination;
	float deltaT;
	long int oldTime;
} IMU_t;

typedef enum {
	IMU_OK = 0,
	IMU_FAIL = 1,
} IMU_Result;

IMU_t IMU;
void IMU_Init(IMU_t *IMU);
void IMU_SetData(IMU_t *IMU, LIS3MDL_MAG *M, LSM6DS3_Data_Bias *A);
void IMU_Update(IMU_t *IMU, LIS3MDL_MAG *M, LSM6DS3_Data_Bias *A);
void IMU_Madgwick(IMU_t *IMU);
void IMU_UpdateRpy(IMU_t *IMU);



#endif /* INC_IMU_H_ */
