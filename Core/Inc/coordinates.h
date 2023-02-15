/*
 * coordinates.h
 *
 *  Created on: Feb 9, 2023
 *      Author: tima
 */

#ifndef INC_COORDINATES_H_
#define INC_COORDINATES_H_

#include "math.h"

#define DEG2RAD M_PI/180.0
#define RAD2DEG 180.0/M_PI

#define EARTH_RADIUS (6371.0 * 1000.0) // meters
#define ACTUAL_GRAVITY 9.80665

typedef struct {
	double lat;
	double lon;
} GeoPoint;

double geoDistanceMeters(double lon1, double lat1, double lon2, double lat2, double alt);
double CoordDistanceBetweenPointsMeters(double lat1, double lon1, double lat2, double lon2, double alt);
double CoordLongitudeToMeters(double lon1, double lon2, double alt);
double CoordLatitudeToMeters(double lat1, double lat2, double alt);
GeoPoint CoordMetersToGeopoint(GeoPoint point, double lonMeters, double latMeters);



#endif /* INC_COORDINATES_H_ */
