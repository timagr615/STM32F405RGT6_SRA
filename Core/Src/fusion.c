/*
 * fusion.c
 *
 *  Created on: 13 февр. 2023 г.
 *      Author: tima
 */

#include "fusion.h"
#include "coordinates.h"


void initFusion(Fusion_t *Fusion, KF_t *KF, uint8_t state_dim, uint8_t obs_dim, uint8_t u_dim, double pp_init, double pv_init, double sigmA){
	InitKF(KF, state_dim, obs_dim, u_dim, pp_init, pv_init, sigmA);
	Fusion->KF = KF;
	Fusion->dt = 0.1;
	Fusion->dx = 0.0;
	Fusion->dy = 0.0;
	Fusion->dx_first = 0.0;
	Fusion->dy_first = 0.0;
}

void Fusion_setFirstPoint(Fusion_t *Fusion, GEO_Point *firstPoint){
	Fusion->dx_first = CoordLongitudeToMeters(firstPoint->lon, 0.0, 0.0);
	Fusion->dy_first = CoordLatitudeToMeters(firstPoint->lat, 0.0, 0.0);
}
void Fusion_computeDxDy(Fusion_t *Fusion, GEO_Point *newPoint){
	Fusion->dx = CoordLongitudeToMeters(newPoint->lon, 0.0, 0.0) - Fusion->dx_first;
	Fusion->dy = CoordLatitudeToMeters(newPoint->lat, 0.0, 0.0) - Fusion->dy_first;
}
void Fusion_compute(Fusion_t *Fusion, IMU_t *imu, GEO_Point *newPoint, double dt){
	uint8_t gps_status = 0;
	if (newPoint->isNew == 1 && newPoint->success == 1){
		gps_status = 1;
		Fusion_computeDxDy(Fusion, newPoint);
	}
	updateKF(Fusion->KF, dt, imu->earth_acc[0], imu->earth_acc[1], Fusion->dx, Fusion->dy, newPoint->speedN, newPoint->speedE, newPoint->hAcc, newPoint->sAcc, gps_status);
}


