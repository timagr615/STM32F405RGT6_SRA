/*
 * gps.h
 *
 *  Created on: Jan 23, 2023
 *      Author: tima
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#define GPS_DEBUG	1
#define GPSBUFFERSIZE 128
#define GPS_FILTER 0.5

typedef struct {

	UART_HandleTypeDef *neo6_huart;
	uint8_t rx_data;
	uint8_t rx_buffer[GPSBUFFERSIZE];
	uint8_t rx_index;
	uint8_t Wait;

	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
	uint8_t Day;
	uint8_t Month;
	uint8_t Year;
    // calculated values
	float PrevLat;
	float PrevLon;
    float dec_longitude;
    float dec_latitude;
    float altitude_ft;

    // GGA - Global Positioning System Fixed Data
    float nmea_longitude;
    float nmea_latitude;
    int utc_time;
    char ns, ew;
    int lock;
    int satelites;
    float hdop;
    float msl_altitude;
    char msl_units;

    // RMC - Recommended Minimmum Specific GNS Data
    char rmc_status;
    float speed_k;
    float course_d;
    int date;

    // GLL
    char gll_status;

    // VTG - Course over ground, ground speed
    float course_t; // ground speed true
    char course_t_unit;
    float course_m; // magnetic
    char course_m_unit;
    char speed_k_unit;
    float speed_km_p;
    float speed_km; // speek km/hr
    char speed_km_unit;

    // GSA
    uint8_t FixMode;
} GPS_t;

GPS_t GPSState;

#if (GPS_DEBUG == 1)
void GPS_print(char *data);
#endif

void GPS_Init(UART_HandleTypeDef *huart);
void GSP_USBPrint(char *data);
void GPS_print_val(char *data, int value);
void GPS_UART_CallBack();
int GPS_validate(char *nmeastr);
void GPS_parse(char *GPSstrParse);
float GPS_nmea_to_dec(float deg_coord, char nsew);
void parseGPGGA(char *GPSstrParse);
void parseGPRMC(char *GPSstrParse);
void parseGPGLL(char *GPSstrParse);
void parseGPVTG(char *GPSstrParse);
void parseGPGSA(char *GPSstrParse);

uint8_t GPS_IsFix();
uint8_t GPS_FixMode();
void GPS_Proccess();

#endif /* INC_GPS_H_ */
