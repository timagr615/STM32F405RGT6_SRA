/*
 * fusion.h
 *
 *  Created on: 13 февр. 2023 г.
 *      Author: tima
 */

#ifndef INC_FUSION_H_
#define INC_FUSION_H_

#include "KF.h"
#include "GNSS.h"
#include "imu.h"

typedef struct {
	KF_t *KF;
	double dt;
	double dx_first;
	double dy_first;

	double dx;
	double dy;

} Fusion_t;

Fusion_t Fusion;

void initFusion(Fusion_t *Fusion, KF_t *KF, uint8_t state_dim, uint8_t obs_dim, uint8_t u_dim, double pp_init, double pv_init, double sigmA);
void Fusion_setFirstPoint(Fusion_t *Fusion, GEO_Point *firstPoint);
void Fusion_computeDxDy(Fusion_t *Fusion, GEO_Point *newPoint);
void Fusion_compute(Fusion_t *Fusion, IMU_t *imu, GEO_Point *newPoint, double dt);



#endif /* INC_FUSION_H_ */
