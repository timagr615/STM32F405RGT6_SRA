/*
 * coordinates.c
 *
 *  Created on: Feb 9, 2023
 *      Author: tima
 */

#include "coordinates.h"
//#include "utils.h"

//extern UART_HandleTypeDef huart1;


static const double a = 6378137.0;   // meter
static const double f = 1 / 298.257223563;
static const double b = (1 - f) * a; // meter


double geoDistanceMeters(double lon1, double lat1, double lon2, double lat2, double alt) {
	double deltaLon = DEG2RAD*(lon2 - lon1);
	double deltaLat = DEG2RAD*(lat2 - lat1);
	double a = pow(sin(deltaLat / 2.0), 2.0) + cos(DEG2RAD*(lat1))*cos(DEG2RAD*(lat2))*pow(sin(deltaLon / 2.0), 2.0);
	double c = 2.0 * atan2(sqrt(a), sqrt(1.0-a));

	 /*char ok[] = "\n\r";
			  HAL_UART_Transmit(&huart1, ok, strlen((char *)ok), 0xFFFF);
	uint8_t data[100];
	print_float(data, deltaLon);
	print_float(data, deltaLat);
	print_float(data, a);
	print_float(data, c);*/
	return (EARTH_RADIUS + alt) * c;
}


double CoordLongitudeToMeters(double lon1, double lon2, double alt) {
	double distance = geoDistanceMeters(lon1, 0.0, lon2, 0.0, alt);
	return distance * (lon1 < 0.0 ? -1.0 : 1.0);
}
//////////////////////////////////////////////////////////////////////////

double CoordLatitudeToMeters(double lat1, double lat2, double alt) {
	double distance = geoDistanceMeters(0.0, lat1, 0.0, lat2, alt);
	return distance * (lat1 < 0.0 ? -1.0 : 1.0);
}


static GeoPoint getPointAhead(GeoPoint point, double distance, double azimuthDegrees) {

	GeoPoint res;
	double radiusFraction = distance / EARTH_RADIUS;
	double bearing = DEG2RAD*azimuthDegrees;
	double lat1 = DEG2RAD*point.lat;
	double lng1 = DEG2RAD*point.lon;

	double lat2_part1 = sin(lat1) * cos(radiusFraction);
	double lat2_part2 = cos(lat1) * sin(radiusFraction) * cos(bearing);
	double lat2 = asin(lat2_part1 + lat2_part2);

	double lng2_part1 = sin(bearing) * sin(radiusFraction) * cos(lat1);
	double lng2_part2 = cos(radiusFraction) - sin(lat1) * sin(lat2);
	double lng2 = lng1 + atan2(lng2_part1, lng2_part2);
	lng2 = fmod(lng2 + 3.0*M_PI, 2.0*M_PI) - M_PI;

	res.lat = RAD2DEG*lat2;
	res.lon = RAD2DEG*lng2;
	return res;
}

static GeoPoint pointPlusDistanceEast(GeoPoint point, double distance) {
	return getPointAhead(point, distance, 90.0);
}

static GeoPoint pointPlusDistanceNorth(GeoPoint point, double distance) {
	return getPointAhead(point, distance, 0.0);
}

GeoPoint CoordMetersToGeopoint(GeoPoint point, double lonMeters, double latMeters) {
	//GeoPoint point = {0.0, 0.0};
	GeoPoint pointEast = pointPlusDistanceEast(point, lonMeters);
	GeoPoint pointNorthEast = pointPlusDistanceNorth(pointEast, latMeters);
	return pointNorthEast;
}
//////////////////////////////////////////////////////////////////////////

double CoordDistanceBetweenPointsMeters(double lat1, double lon1,
                                        double lat2, double lon2, double alt) {
  return geoDistanceMeters(lon1, lat1, lon2, lat2, alt);
}



