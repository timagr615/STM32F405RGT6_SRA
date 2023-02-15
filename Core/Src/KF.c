/*
 * KF.c
 *
 *  Created on: Feb 10, 2023
 *      Author: tima
 */

#include "KF.h"
#include <math.h>

void InitKF(KF_t *KF, uint8_t state_dim, uint8_t obs_dim, uint8_t u_dim, double p, double v, double sigmA){
	alloc_filter(KF, state_dim, obs_dim, u_dim, sigmA);
	KF_initP(KF, p, v);
	KF_initH(KF);
}


void alloc_filter(KF_t *KF, uint8_t state_dim, uint8_t obs_dim, uint8_t u_dim, double sigmA){
	//double b[state_dim*obs_dim] = {1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0};
	KF->sigmAcc = sigmA;
	KF->obs_dim = obs_dim;
	KF->state_dim = state_dim;
	KF->state = vector_new(state_dim);
	KF->u = vector_new(u_dim);
	KF->Hx = vector_new(obs_dim);
	KF->z = vector_new(obs_dim);
	KF->y = vector_new(obs_dim);
	KF->Ky = vector_new(state_dim);

	KF->P = matrix_eye(state_dim);
	KF->Q = matrix_new(state_dim, state_dim);
	KF->R = matrix_eye(obs_dim);

	KF->A = matrix_eye(state_dim);
	KF->B = matrix_new(state_dim, u_dim);
	//KF->K = matrix_new(state_dim, obs_dim);
	KF->H = matrix_new(obs_dim, state_dim);
}

void free_filter(KF_t *KF){
	vector_free(KF->state);
	vector_free(KF->u);
	vector_free(KF->Hx);
	vector_free(KF->z);
	vector_free(KF->y);
	vector_free(KF->Ky);

	matrix_free(KF->P);
	matrix_free(KF->Q);
	matrix_free(KF->R);

	matrix_free(KF->A);
	matrix_free(KF->B);
	//matrix_free(KF->K);
	matrix_free(KF->H);
}

void KF_initP(KF_t *KF, double p, double v){
	matrix_set_el(KF->P, 0, 0, p);
	matrix_set_el(KF->P, 1, 1, p);
	matrix_set_el(KF->P, 2, 2, v);
	matrix_set_el(KF->P, 3, 3, v);
}

void KF_initH(KF_t *KF){
	matrix_set_el(KF->H, 0, 0, 1.0);
	matrix_set_el(KF->H, 1, 1, 1.0);
	matrix_set_el(KF->H, 2, 2, 1.0);
	matrix_set_el(KF->H, 3, 3, 1.0);
}

void KF_updateA(KF_t *KF, double dt)
{
	matrix_set_el(KF->A, 0, 2, dt);
	matrix_set_el(KF->A, 1, 3, dt);
}
void KF_updateB(KF_t *KF, double dt){
	double dtt = dt*dt/2;
	matrix_set_el(KF->B, 0, 0, dtt);
	matrix_set_el(KF->B, 1, 1, dtt);
	matrix_set_el(KF->B, 2, 0, dt);
	matrix_set_el(KF->B, 3, 1, dt);
}

void KF_updateQ(KF_t *KF, double dt){
	double dt4 = KF->sigmAcc*pow(dt, 4)/4.0;
	double dt3 = KF->sigmAcc*pow(dt, 3)/2.0;
	double dt2 = KF->sigmAcc*dt*dt;
	matrix_set_el(KF->Q, 0, 0, dt4);
	matrix_set_el(KF->Q, 0, 2, dt3);
	matrix_set_el(KF->Q, 1, 1, dt4);
	matrix_set_el(KF->Q, 1, 3, dt3);
	matrix_set_el(KF->Q, 2, 0, dt3);
	matrix_set_el(KF->Q, 2, 2, dt2);
	matrix_set_el(KF->Q, 3, 1, dt3);
	matrix_set_el(KF->Q, 3, 3, dt2);
}

void KF_updateR(KF_t *KF, double varP, double varV){
	double vp_sq = varP*varP;
	double vv_sq = varV*varV;
	matrix_set_el(KF->R, 0, 0, vp_sq);
	matrix_set_el(KF->R, 1, 1, vp_sq);
	matrix_set_el(KF->R, 2, 2, vv_sq);
	matrix_set_el(KF->R, 3, 3, vv_sq);
}


/* --------------PREDICTION-----------------------
 * x(k+1) = A*x(k) + B*u(k)
 * P(k+1) = A*P(k)*A_T + Q
 */

void predicKF(KF_t *KF, double dt, double an, double ae){
	//update u, A, B
	vector_set_el(KF->u, 0, an);
	vector_set_el(KF->u, 1, ae);
	KF_updateA(KF, dt);
	KF_updateB(KF, dt);
	KF_updateQ(KF, dt);

	// x = Ax+Bu
	vector_t *ax = matrix_mult_vector(KF->A, KF->state);
	vector_t *bu = matrix_mult_vector(KF->B, KF->u);
	vector_add_vector_ip(ax, bu);
	vector_from_vector_ip(KF->state, ax);


	vector_free(ax);
	vector_free(bu);

	//P(k+1) = A*P(k)*A_T + Q
	matrix_t *at = matrix_transpose(KF->A);
	matrix_t *pat = matrix_dot(KF->P, at);
	matrix_free(at);
	matrix_t *apat = matrix_dot(KF->A, pat);
	matrix_free(pat);
	matrix_add_ip(apat, KF->Q);
	matrix_from_matrix_ip(KF->P, apat);
	matrix_free(apat);
	matrix_add_ip(KF->P, KF->Q);

}

void estimateKF(KF_t *KF, double px, double py, double vx, double vy){
	//K(k) = P(k)*H_T*(H*P(k)*H_T + R)^-1
	matrix_t *ht = matrix_transpose(KF->H);
	matrix_t *pht = matrix_dot(KF->P, ht);
	matrix_t *hpht = matrix_dot(KF->H, pht);
	matrix_add_ip(hpht, KF->R);
	matrix_t *hpht_inv = matrix_invert(hpht);
	matrix_t *hthpht_inv = matrix_dot(ht, hpht_inv);
	matrix_t *K = matrix_dot(KF->P, hthpht_inv);

	//x(k) = x(k) + K(k)*(z(k) - H*x(k))
	vector_t *Hx = matrix_mult_vector(KF->H, KF->state);
	double zz[] = {px, py, vx, vy};
	vector_t *z = vector_from_arr(KF->obs_dim, zz);
	vector_sub_vector_ip(z, Hx);
	vector_t *y = matrix_mult_vector(K, z);
	vector_add_vector_ip(KF->state, y);

	//P(k) = (I - K(k)*H)*P(k)
	matrix_t *KH = matrix_dot(K, KF->H);
	matrix_t *I = matrix_eye(KF->state_dim);
	matrix_sub_ip(I, KH);
	matrix_t *P_new = matrix_dot(I, KF->P);
	matrix_from_matrix_ip(KF->P, P_new);

	matrix_free(pht);
	matrix_free(hpht);
	matrix_free(ht);
	matrix_free(hpht_inv);
	matrix_free(hthpht_inv);
	matrix_free(K);
	vector_free(Hx);
	vector_free(z);
	vector_free(y);
	matrix_free(KH);
	matrix_free(I);
	matrix_free(P_new);

}

void updateKF(KF_t *KF, double dt, double an, double ae, double px, double py, double vx, double vy, double varP, double varV, uint8_t gps_status){

	predicKF(KF, dt, an, ae);
	if (gps_status == 1){
		KF_updateR(KF, varP, varV);
		estimateKF(KF, px, py, vx, vy);
	}
}



