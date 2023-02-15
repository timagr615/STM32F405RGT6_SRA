/*
 * imu.c
 *
 *  Created on: Feb 3, 2023
 *      Author: tima
 */
#include <stdio.h>
#include "math.h"
#include "imu.h"

#include "micros.h"

float beta = sqrt(3.0 / 4.0) * M_PI * (40.0 / 180.0);

void IMU_Init(IMU_t *IMU){
	for (uint8_t i = 0; i< 3; i++){
		IMU->ap[i] = 0.0;
		IMU->a[i] = 0.0;
		IMU->gp[i] = 0.0;
		IMU->g[i] = 0.0;
		IMU->m[i] = 0.0;

		IMU->aned[i] = 0.0;
		IMU->gned[i] = 0.0;
		IMU->mned[i] = 0.0;

		IMU->q[i+1] = 0.0;
		IMU->rpyp[i] = 0.0;
		IMU->rpy[i] = 0.0;
		IMU->lin_acc[i] = 0.0;
		IMU->earth_acc[i] = 0.0;
		IMU->earth_gyro[i] = 0.0;
	}
	IMU->q[0] = 1.0;
	IMU->long_acc = 0.0;
	IMU->n_filter_iter = 15;
	IMU->magnetic_declination = MAGNETIC_DECLINATION;
	IMU->deltaT = 0.0;
	IMU->oldTime = 0;
}
void IMU_SetData(IMU_t *IMU, LIS3MDL_MAG *M, LSM6DS3_Data_Bias *A){
	for (uint8_t i = 0; i < 3; i++){
		IMU->a[i] = IMU->ap[i] + ACC_LPF*(A->a[i] - IMU->ap[i]);
		IMU->g[i] = IMU->gp[i] + GYRO_LPF*(A->g[i] - IMU->gp[i]);
		IMU->m[i] = M->m[i];
		IMU->ap[i] = IMU->a[i];
		IMU->gp[i] = IMU->g[i];
	}
	//printf("in set data: %.2f %.2f %.2f \r\n", A->a[0], A->a[1], A->a[2]);

	IMU->aned[0] = -IMU->a[0];
	IMU->aned[1] = IMU->a[1];
	IMU->aned[2] = IMU->a[2];

	IMU->gned[0] = IMU->g[0]* DEG_TO_RAD;
	IMU->gned[1] = -IMU->g[1]* DEG_TO_RAD;
	IMU->gned[2] = -IMU->g[2]* DEG_TO_RAD;

	IMU->mned[0] = IMU->m[0];
	IMU->mned[1] = -IMU->m[1];
	IMU->mned[2] = -IMU->m[2];
}
void IMU_Update(IMU_t *IMU, LIS3MDL_MAG *M, LSM6DS3_Data_Bias *A){
	IMU_SetData(IMU, M, A);
	for (uint8_t i = 0; i < IMU->n_filter_iter; i++){
		IMU_Madgwick(IMU);
	}

	IMU_UpdateRpy(IMU);
}
void IMU_Madgwick(IMU_t *IMU){
	long int newTime = micros();
	long int delta = newTime - IMU->oldTime;
	IMU->oldTime = newTime;
	IMU->deltaT = fabs(delta * 0.001 * 0.001);
	//printf("deltaT: %f \r\n", IMU->deltaT);
	double q0 = IMU->q[0], q1 = IMU->q[1], q2 = IMU->q[2], q3 = IMU->q[3];  // short name local variable for readability
	double gx = IMU->gned[0], gy = IMU->gned[1], gz = IMU->gned[2];
	double ax = IMU->aned[0]*1000.0, ay = IMU->aned[1]*1000.0, az = IMU->aned[2]*1000.0;
	double mx = IMU->mned[0]*1000.0, my = IMU->mned[1]*1000.0, mz = IMU->mned[2]*1000.0;
	double recipNorm;
	double s0, s1, s2, s3;
	double qDot1, qDot2, qDot3, qDot4;
	double hx, hy;
	double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Normalise accelerometer measurement
	double a_norm = ax * ax + ay * ay + az * az;
	if (a_norm == 0.) return;  // handle NaN
	recipNorm = 1.0 / sqrt(a_norm);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;


	// Normalise magnetometer measurement
	double m_norm = mx * mx + my * my + mz * mz;
	if (m_norm == 0.) return;  // handle NaN
	recipNorm = 1.0 / sqrt(m_norm);
	mx *= recipNorm;
	my *= recipNorm;
	mz *= recipNorm;

	// Auxiliary variables to avoid repeated arithmetic
	_2q0mx = 2.0f * q0 * mx;
	_2q0my = 2.0f * q0 * my;
	_2q0mz = 2.0f * q0 * mz;
	_2q1mx = 2.0f * q1 * mx;
	_2q0 = 2.0f * q0;
	_2q1 = 2.0f * q1;
	_2q2 = 2.0f * q2;
	_2q3 = 2.0f * q3;
	_2q0q2 = 2.0f * q0 * q2;
	_2q2q3 = 2.0f * q2 * q3;
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;

	// Reference direction of Earth's magnetic field
	hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
	hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;
	// Gradient decent algorithm corrective step
	s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	//printf("IMU q %f %f %f %f \r\n", s0, s1, s2, s3);

	recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // normalise step magnitude
	s0 *= recipNorm;
	s1 *= recipNorm;
	s2 *= recipNorm;
	s3 *= recipNorm;
	//printf("IMU q %f %f %f %f \r\n", s0, s1, s2, recipNorm);

	// Apply feedback step
	qDot1 -= beta * s0;
	qDot2 -= beta * s1;
	qDot3 -= beta * s2;
	qDot4 -= beta * s3;
	//printf("IMU q %f %f %f %f \r\n", qDot1, qDot2, qDot3, qDot4);

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * IMU->deltaT;
	q1 += qDot2 * IMU->deltaT;
	q2 += qDot3 * IMU->deltaT;
	q3 += qDot4 * IMU->deltaT;
	//printf("IMU q %f %f %f %f \r\n", q0, q1, q2, q3);

	// Normalise quaternion
	recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	IMU->q[0] = q0;
	IMU->q[1] = q1;
	IMU->q[2] = q2;
	IMU->q[3] = q3;
	//printf("IMU q %f %f %f %f \r\n", IMU->q[0], IMU->q[1], IMU->q[2], IMU->q[3]);
}
void IMU_UpdateRpy(IMU_t *IMU){
	float qw = IMU->q[0], qx = IMU->q[1], qy = IMU->q[2], qz = IMU->q[3];
	float a11, a12, a13, a21, a22, a23, a31, a32, a33;  // rotation matrix coefficients for Euler angles and gravity components
	float rpy0, rpy1, rpy2;
	a11 = 2.0f * (qw*qw +qx*qx - 0.5f);
	a12 = 2.0f * (qx*qy - qw*qz);
	a13 = 2.0f * (qw*qy + qx*qz);
	a21 = 2.0f * (qx*qy + qw*qz);
	a22 = 2.0f * (qw*qw + qy*qy - 0.5f);
	a23 = 2.0f * (qy*qz - qw*qx);
	a31 = 2.0f * (qx*qz - qw*qy);
	a32 = 2.0f * (qw*qx + qy*qz);
	a33 = 2.0f * (qw*qw + qz*qz -0.5f);
	rpy0 = atan2f(a32, a33);
	rpy1 = -asinf(a31);
	rpy2 = atan2f(a21, a11);
		        /*MPU9250->rpy[0] = atan2f(a31, a33);
		        MPU9250->rpy[1] = -asinf(a32);
		        MPU9250->rpy[2] = atan2f(a12, a22);*/
	rpy0 *= 180.0f / M_PI;
	rpy1 *= 180.0f / M_PI;
	rpy2 *= 180.0f / M_PI;
	rpy2 += IMU->magnetic_declination;
	if (rpy2 < 0) rpy2 += 360.f;
		        /*if (MPU9250->rpy[2] >= +180.f)
		        	MPU9250->rpy[2] -= 360.f;
		        else if (MPU9250->rpy[2] < -180.f)
		        	MPU9250->rpy[2] += 360.f;*/
	IMU -> rpy[0] = IMU -> rpyp[0] + ROLL_FILTER*(rpy0 - IMU -> rpyp[0]);
	IMU -> rpy[1] = IMU -> rpyp[1] + PITCH_FILTER*(rpy1 - IMU -> rpyp[1]);
	IMU -> rpy[2] = IMU -> rpyp[2] + YAW_FILTER*(rpy2 - IMU -> rpyp[2]);

	IMU->rpyp[0] = IMU->rpy[0];
	IMU->rpyp[1] = IMU->rpy[1];
	IMU->rpyp[2] = IMU->rpy[2];

	/*IMU->lin_acc[0] = -IMU->a[1] + a31;
	IMU->lin_acc[1] = +IMU->a[0] + a32;
	IMU->lin_acc[2] = -IMU->a[2] - a33;*/
	IMU->lin_acc[0] = -IMU->aned[0] + a31;
	IMU->lin_acc[1] = -IMU->aned[1] + a32;
	IMU->lin_acc[2] = IMU->aned[2] - a33;

	IMU->earth_acc[0] = a11*IMU->lin_acc[0] + a12*IMU->lin_acc[1] + a13*IMU->lin_acc[2];
	IMU->earth_acc[1] = a21*IMU->lin_acc[0] + a22*IMU->lin_acc[1] + a23*IMU->lin_acc[2];
	IMU->earth_acc[2] = a31*IMU->lin_acc[0] + a32*IMU->lin_acc[1] + a33*IMU->lin_acc[2];

	IMU->earth_gyro[0] = (a11*IMU->gned[0] + a12*IMU->gned[1] + a13*IMU->gned[2])* DEG_TO_RAD;
	IMU->earth_gyro[1] = (a21*IMU->gned[0] + a22*IMU->gned[1] + a23*IMU->gned[2])* DEG_TO_RAD;
	IMU->earth_gyro[2] = (a31*IMU->gned[0] + a32*IMU->gned[1] + a33*IMU->gned[2])* DEG_TO_RAD;

	IMU->long_acc = sqrt(IMU->earth_acc[0]*IMU->earth_acc[0]+IMU->earth_acc[1]*IMU->earth_acc[1]);

}

