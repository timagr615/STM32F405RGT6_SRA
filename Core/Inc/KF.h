/*
 * KF.h
 *
 *  Created on: Feb 10, 2023
 *      Author: tima
 */

#ifndef INC_KF_H_
#define INC_KF_H_

#include "matrix.h"
#include "stdint.h"

/*
 * INITIALIZING P, Q, R
 *
 * --------------PREDICTION-----------------------
 * x(k+1) = A*x(k) + B*u(k)
 * P(k+1) = A*P(k)*A_T + Q
 *
 * --------------CORRECTION-----------------------
 *
 * Kalman gain
 * K(k) = P(k)*H_T*(H*P(k)*H_T + R)^-1
 *
 * update estimate
 * x(k) = x(k) + K(k)*(z(k) - H*x(k))
 *
 * update error covariance
 *
 * P(k) = (I - K(k)*H)*P(k)
 */

//#define sa 0.1

typedef struct {
	uint8_t state_dim;
	uint8_t obs_dim;
	double sigmAcc;
	vector_t *state;
	vector_t *u;
	vector_t *Hx;
	vector_t *z;
	vector_t *y;
	vector_t *Ky;

	matrix_t *P;
	matrix_t *Q;
	matrix_t *R;

	matrix_t *A;
	matrix_t *B;
	matrix_t *K;
	matrix_t *H;
} KF_t;

KF_t KF;

void InitKF(KF_t *KF, uint8_t state_dim, uint8_t obs_dim, uint8_t u_dim, double p, double v, double sigmA);
void alloc_filter(KF_t *KF, uint8_t state_dim, uint8_t obs_dim, uint8_t u_dim, double sigmA);
void free_filter(KF_t *KF);
void KF_initP(KF_t *KF, double p, double v);
void KF_initH(KF_t *KF);


void KF_updateA(KF_t *KF, double dt);
void KF_updateB(KF_t *KF, double dt);
void KF_updateQ(KF_t *KF, double dt);
void KF_updateR(KF_t *KF, double varP, double varV);

void predicKF(KF_t *KF, double dt, double an, double ae);
void estimateKF(KF_t *KF, double px, double py, double vx, double vy);

void updateKF(KF_t *KF, double dt, double an, double ae, double px, double py, double vx, double vy, double varP, double varV, uint8_t gps_status);



#endif /* INC_KF_H_ */
