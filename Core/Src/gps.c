/*
 * gps.c
 *
 *  Created on: Jan 23, 2023
 *      Author: tima
 */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include "gps.h"
#include "stdlib.h"




/*uint8_t rx_data = 0;
uint8_t rx_buffer[GPSBUFFERSIZE];
uint8_t rx_index = 0;*/



#if (GPS_DEBUG == 1)
void GPS_print(char *data){
	char buf[GPSBUFFERSIZE] = {0,};
	sprintf(buf, "%s\n\r", data);
	printf(buf);
	printf("\r\n");
}
#endif

void GPS_Init(UART_HandleTypeDef *huart)
{
	GPSState.neo6_huart = huart;
	GPSState.rx_data = 0;
	GPSState.rx_index = 0;
	GPSState.Wait = 1;
	// calculated values
	/*GPSState.dec_longitude = 0.0;
	GPSState.dec_latitude = 0.0;
	GPSState.altitude_ft = 0.0;

	    // GGA - Global Positioning System Fixed Data
	GPSState.nmea_longitude = 0.0;
	GPSState.nmea_latitude = 0.0;
	GPSState.utc_time = 0.0;
	GPSState.ns = '0';
	GPSState.ew = '0';
	GPSState.lock = 0;
	GPSState.satelites = 0;
	GPSState.hdop = 0.0;
	GPSState.msl_altitude = 0.0;
	GPSState.msl_units = '0';

	    // RMC - Recommended Minimmum Specific GNS Data
	GPSState.rmc_status = '0';
	GPSState.speed_k = 0.0;
	GPSState.course_d = 0.0;
	GPSState.date  = 0;

	    // GLL
	GPSState.gll_status = '0';

	    // VTG - Course over ground, ground speed
	GPSState.course_t = 0.0; // ground speed true
	GPSState.course_t_unit = '0';
	GPSState.course_m = 0.0; // magnetic
	GPSState.course_m_unit = '0';
	GPSState.speed_k_unit = '0';
	GPSState.speed_km = 0.0; // speek km/hr
	GPSState.speed_km_unit = '0';*/
	HAL_UART_Receive_IT(GPSState.neo6_huart, &GPSState.rx_data, 1);
}


void GPS_UART_CallBack(){

	if (GPSState.rx_data != '\n' && GPSState.rx_index < sizeof(GPSState.rx_buffer)) {
		GPSState.rx_buffer[GPSState.rx_index++] = GPSState.rx_data;
	} else {
		printf("GPS callback \r\n");
		#if (GPS_DEBUG == 1)
		printf("%s \r\n", GPSState.rx_data);
		GPS_print((char*)(GPSState.rx_buffer));
		#endif


		/*if(GPS_validate((char*) GPSState.rx_buffer))
			GPS_parse((char*) GPSState.rx_buffer);
		//GPS_parse((char*) rx_buffer);
		GPSState.rx_index = 0;
		memset(GPSState.rx_buffer, 0, sizeof(GPSState.rx_buffer));*/
	}
	HAL_UART_Receive_IT(GPSState.neo6_huart, &GPSState.rx_data, 1);
}

void GPS_Proccess()
{
	if(GPS_validate((char*) GPSState.rx_buffer))
		GPS_parse((char*) GPSState.rx_buffer);
	//GPS_parse((char*) rx_buffer);
	GPSState.rx_index = 0;
	memset(GPSState.rx_buffer, 0, sizeof(GPSState.rx_buffer));
}

int GPS_validate(char *nmeastr){
    char check[3];
    char checkcalcstr[3];
    int i;
    int calculated_check;

    i=0;
    calculated_check=0;

    // check to ensure that the string starts with a $
    if(nmeastr[i] == '$')
        i++;
    else
        return 0;

    //No NULL reached, 75 char largest possible NMEA message, no '*' reached
    while((nmeastr[i] != 0) && (nmeastr[i] != '*') && (i < 75)){
        calculated_check ^= nmeastr[i];// calculate the checksum
        i++;
    }

    if(i >= 75){
        return 0;// the string was too long so return an error
    }

    if (nmeastr[i] == '*'){
        check[0] = nmeastr[i+1];    //put hex chars in check string
        check[1] = nmeastr[i+2];
        check[2] = 0;
    }
    else
        return 0;// no checksum separator found there for invalid

    sprintf(checkcalcstr,"%02X",calculated_check);
    return((checkcalcstr[0] == check[0])
        && (checkcalcstr[1] == check[1])) ? 1 : 0 ;
}

void GPS_parse(char *GPSstrParse){
    if(!strncmp(GPSstrParse, "$GPGGA", 6)){
    	/*if (sscanf(GPSstrParse, "$GPGGA,%*f,%*f,%*c,%*f,%*c,%*d,%d*,%*f,%*f,%*c", &GPSState.utc_time, &GPSState.nmea_latitude, &GPSState.ns, &GPSState.nmea_longitude, &GPSState.ew, &GPSState.lock, &GPSState.satelites, &GPSState.hdop, &GPSState.msl_altitude, &GPSState.msl_units) >= 1){
    		GPSState.dec_latitude = GPS_nmea_to_dec(GPSState.nmea_latitude, GPSState.ns);
    		GPSState.dec_longitude = GPS_nmea_to_dec(GPSState.nmea_longitude, GPSState.ew);
    		return;
    	}*/
    	parseGPGGA((char*)GPSstrParse);

    	return;
    }
    else if (!strncmp(GPSstrParse, "$GPRMC", 6)){
    	parseGPRMC(GPSstrParse);

    	return;

    }
    else if (!strncmp(GPSstrParse, "$GPGLL", 6)){
    	parseGPGLL(GPSstrParse);
    	return;
    }
    else if (!strncmp(GPSstrParse, "$GPVTG", 6)){
    	parseGPVTG(GPSstrParse);
		return;
    }
    else if (!strncmp(GPSstrParse, "$GPGSA", 6)){
        	parseGPGSA(GPSstrParse);
    		return;
    }
    if (GPSState.ns != "?" && GPSState.ew != "?" )
    {
    	float dec_lat = GPS_nmea_to_dec(GPSState.nmea_latitude, GPSState.ns);
    	float dec_lon = GPS_nmea_to_dec(GPSState.nmea_longitude, GPSState.ew);
    	//printf("dec_lat_lon %.4f %.4f %.2f \r\n", dec_lat, dec_lon, GPSState.course_t);
    	if (GPSState.Wait == 1)
    	{
    		GPSState.dec_latitude = dec_lat;
    		GPSState.dec_longitude = dec_lon;
    	}
    	else
    	{
    		if (fabs(GPSState.PrevLat-dec_lat)<0.01 && fabs(GPSState.PrevLon-dec_lon)<0.01)
    		{
    			GPSState.PrevLat = GPSState.dec_latitude;
    			GPSState.PrevLon = GPSState.dec_longitude;
    			GPSState.dec_latitude = dec_lat;
    			GPSState.dec_longitude = dec_lon;
    		}
    	}
    	//printf("dec_lat_lon %.4f %.4f %.4f %.4f \r\n", GPSState.dec_latitude, GPSState.dec_longitude, GPSState.PrevLat,GPSState.PrevLon);
    }
}

float GPS_nmea_to_dec(float deg_coord, char nsew) {
    int degree = (int)(deg_coord/100);
    float minutes = deg_coord - degree*100;
    float dec_deg = minutes / 60;
    float decimal = degree + dec_deg;
    if (nsew == 'S' || nsew == 'W') { // return negative
        decimal *= -1;
    }
    return decimal;
}
void parseGPGGA(char *GPSstrParse)
{

	char *p = strchr(GPSstrParse, ',');

	/*GPSState.utc_time = atoi(p+1);
	GPSState.Second = GPSState.utc_time % 100;
	GPSState.Minute = (GPSState.utc_time/100) % 100;
	GPSState.Hour = (GPSState.utc_time/10000) % 100;*/

	p = strchr(p+1, ',');
	GPSState.nmea_latitude = atof(p+1);

	p = strchr(p+1, ',');
	GPSState.ns = p[1] == "," ? "?" : p[1];

	p = strchr(p+1, ',');
	GPSState.nmea_longitude = atof(p+1);

	p = strchr(p+1, ',');
	GPSState.ew = p[1] == "," ? "?" : p[1];

	p = strchr(p+1, ',');
	GPSState.lock = atoi(p+1);

	p = strchr(p+1, ',');
	GPSState.satelites = atoi(p+1);

	p = strchr(p+1, ',');
	GPSState.hdop = atof(p+1);

	p = strchr(p+1, ',');
	GPSState.msl_altitude = atof(p+1);

	p = strchr(p+1, ',');
	GPSState.msl_units = p[1] == "," ? "?" : p[1];
}
void parseGPRMC(char *GPSstrParse)
{
	char *p = strchr(GPSstrParse, ',');
	GPSState.utc_time = atoi(p+1);
	//printf("%d \r\n", GPSState.utc_time);
	GPSState.Second = GPSState.utc_time % 100;
	GPSState.Minute = (GPSState.utc_time/100) % 100;
	GPSState.Hour = (GPSState.utc_time/10000) % 100;

	// Navigation receiver warning A = OK, V = warning
	p = strchr(p+1, ',');

	p = strchr(p+1, ',');
	GPSState.nmea_latitude = atof(p+1);

	p = strchr(p+1, ',');
	GPSState.ns = p[1] == "," ? "?" : p[1];

	p = strchr(p+1, ',');
	GPSState.nmea_longitude = atof(p+1);

	p = strchr(p+1, ',');
	GPSState.ew = p[1] == "," ? "?" : p[1];

	p = strchr(p+1, ',');
	GPSState.speed_k = atof(p+1);

	p = strchr(p+1, ',');
	GPSState.course_d = atof(p+1);

	p = strchr(p+1, ',');
	GPSState.date = atoi(p+1);
	GPSState.Year = GPSState.date % 100;
	GPSState.Month = (GPSState.date/100) % 100;
	GPSState.Day = (GPSState.date/10000) % 100;

}
void parseGPGLL(char *GPSstrParse)
{
	char *p = strchr(GPSstrParse, ',');
	GPSState.nmea_latitude = atof(p+1);

	p = strchr(p+1, ',');
	GPSState.ns = p[1] == "," ? "?" : p[1];

	p = strchr(p+1, ',');
	GPSState.nmea_longitude = atof(p+1);

	p = strchr(p+1, ',');
	GPSState.ew = p[1] == "," ? "?" : p[1];

	p = strchr(p+1, ',');
	/*GPSState.utc_time = atoi(p+1);
	GPSState.Second = GPSState.utc_time % 100;
	GPSState.Minute = (GPSState.utc_time/100) % 100;
	GPSState.Hour = (GPSState.utc_time/10000) % 100;*/

	p = strchr(p+1, ',');
	GPSState.gll_status = p[1] == "," ? "?" : p[1];

}
void parseGPVTG(char *GPSstrParse)
{
	float sp;
	char *p = strchr(GPSstrParse, ',');
	GPSState.course_t = atof(p+1);

	p = strchr(p+1, ',');
	GPSState.course_t_unit = p[1] == "," ? "?" : p[1];

	p = strchr(p+1, ',');
	GPSState.course_m = atof(p+1);

	p = strchr(p+1, ',');
	GPSState.course_m_unit = p[1] == "," ? "?" : p[1];

	p = strchr(p+1, ',');
	GPSState.speed_k = atof(p+1);

	p = strchr(p+1, ',');
	GPSState.speed_k_unit = p[1] == "," ? "?" : p[1];

	p = strchr(p+1, ',');
	sp = atof(p+1);
	if (sp == 0.0)
	{
		sp = GPSState.speed_km_p;
	}
	GPSState.speed_km = GPSState.speed_km_p + GPS_FILTER*(sp-GPSState.speed_km_p);
	GPSState.speed_km_p = GPSState.speed_km;

	p = strchr(p+1, ',');
	GPSState.speed_km_unit = p[1] == "," ? "?" : p[1];
}

void parseGPGSA(char *GPSstrParse)
{
	char *p = strchr(GPSstrParse, ',');

	p = strchr(p+1, ',');
	GPSState.FixMode = atoi(p+1);
}

uint8_t GPS_IsFix()
{
	return GPSState.lock;
}
uint8_t GPS_FixMode()
{
	return GPSState.FixMode;
}
