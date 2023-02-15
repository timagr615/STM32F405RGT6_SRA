/*
 * GNSS.c
 *
 *  Created on: 23 янв. 2023 г.
 *      Author: tima
 */
#include "GNSS.h"
#include <stdio.h>
#include <cmsis_os.h>

union u_Short uShort;
union i_Short iShort;
union u_Long uLong;
union i_Long iLong;


void GEO_Point_set(GNSS_StateHandle *GNSS, GEO_Point *point, uint8_t success, uint8_t isNew){
	point->year = GNSS->year;
	point->month = GNSS->month;
	point->day = GNSS->day;
	point->hour = GNSS->hour;
	point->min = GNSS->min;
	point->sec = GNSS->sec;
	point->lon = GNSS->fLon;
	point->lat = GNSS->fLat;
	point->speed = GNSS->gSpeed;
	point->speedN = GNSS->velN;
	point->speedE = GNSS->velE;
	point->course = GNSS->course;
	point->hAcc = GNSS->hAcc;
	point->sAcc = GNSS->sAcc;
	point->success = success;
	point->isNew = isNew;
}

void GEO_Point_update(GEO_Point *old, GEO_Point *point){
	old->year = point->year;
	old->month = point->month;
	old->day = point->day;
	old->hour = point->hour;
	old->min = point->min;
	old->sec = point->sec;
	old->lon = point->lon;
	old->lat = point->lat;
	old->speed = point->speed;
	old->speedN = point->speedN;
	old->speedE = point->speedE;
	old->course = point->course;
	old->hAcc = point->hAcc;
	old->hAcc = point->hAcc;
	old->success = point->success;
	old->isNew = point->isNew;
}

/*!
 * Structure initialization.
 * @param GNSS Pointer to main GNSS structure.
 * @param huart Pointer to uart handle.
 */
void GNSS_Init(GNSS_StateHandle *GNSS, UART_HandleTypeDef *huart) {
	GNSS->huart = huart;
	GNSS->year = 0;
	GNSS->month = 0;
	GNSS->day = 0;
	GNSS->hour = 0;
	GNSS->min = 0;
	GNSS->sec = 0;
	GNSS->timeValid = 0;
	GNSS->fixType = 0;
	GNSS->numSV = 0;
	GNSS->satCount = 0;
	GNSS->lon = 0;
	GNSS->lat = 0;
	GNSS->height = 0;
	GNSS->hMSL = 0;
	GNSS->hAcc = 0;
	GNSS->vAcc = 0;
	GNSS->sAcc = 0;
	GNSS->headAcc = 0;
	GNSS->hDop = 0.0;
	GNSS->nDop = 0.0;
	GNSS->eDop = 0.0;
	GNSS->velN = 0.0;
	GNSS->velE = 0.0;
	GNSS->velD = 0.0;
	GNSS->gSpeed = 0.0;
	GNSS->headMot = 0;
	GNSS->course = 0.0;
}

void GPS_Filter_Init(GNSS_Filter *GPS_Filter){
	GPS_Filter->minFixmode = 2;
	GPS_Filter->maxHacc = 20;
	GPS_Filter->firstFix = 0;

	GPS_Filter->oldFixmode = 0;
	GPS_Filter->oldHacc = 100000;
	GPS_Filter->maxAcceleration = 5.0;
	GPS_Filter->maxSpeed = 12.0;

}

void GPS_FilterUpdate(GNSS_StateHandle *GNSS, GNSS_Filter *GPS_Filter, GEO_Point *po, GEO_Point *pc){
	float acc = 0.0;
	float distance = 0.0;
	float spdM = 0.0;
	uint16_t to = 0;
	uint16_t t1 = 0;


	// Был ли первый фикс
	if(GPS_Filter->firstFix == 1){
		// Соответствует ли фиксмод и точность установленным границам?
		if (GPS_Filter->minFixmode <= GNSS->fixType < 5 && GNSS->hAcc < GPS_Filter->maxHacc){
			GEO_Point_set(GNSS, pc, 0, 0);
			to = po->hour*60*60 + po->min*60 + po->sec;
			t1 = pc->hour*60*60 + pc->min*60 + pc->sec;
			distance = geoDistanceMeters(pc->lon, pc->lat, po->lon, po->lat, 0.0);
			if (to != t1){
				acc = fabs((pc->speed - po->speed)/(t1-to));
				spdM = fabs(distance/(t1-to));
			} else {
				acc = GPS_Filter->maxAcceleration + 1.0;
				spdM = GPS_Filter->maxSpeed + 1.0;
			}


			printf("dist: %.3f old hacc: %.2f  new hacc: %.2f \r\n", distance, po->hAcc, pc->hAcc);
			// Соответствует ли ускорение, скорость, расстояние между точками заданным границам?
			if(to != t1 && acc < GPS_Filter->maxAcceleration &&  pc->speed < GPS_Filter->maxSpeed && spdM < GPS_Filter->maxSpeed){
				if (distance > 0.5*(po->hAcc + pc->hAcc)){
					pc->isNew = 1;
					pc->success = 1;
					GEO_Point_update(po, pc);
				} else {
					pc->isNew = 1;
					pc->success = 1;
					po->hAcc = pc->hAcc;
					po->sAcc = pc->sAcc;
					pc->lat = po->lat;
					pc->lon = po->lon;
					pc->speed = 0.0;
					pc->speedN = 0.0;
					pc->speedE = 0.0;
				}

			} else {
				po->hAcc = pc->hAcc;
				po->sAcc = pc->sAcc;
				GEO_Point_update(pc, po);
				pc->isNew = 0;
				pc->success = 0;
			}
		} else {
			//GEO_Point_update(&pointCurrent, &pointOld);
			pc->isNew = 0;
			pc->success = 0;
		}
	}
}

/*!
 * Searching for a header in data buffer and matching class and message ID to buffer data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParseBuffer(GNSS_StateHandle *GNSS) {
	/*printf("PARSE BUFFER \r\n");
	for(uint8_t i = 0; i < 100; i++){
		printf("0x%X ", GNSS->uartWorkingBuffer[i]);
	}
	printf("\r\n\r\n");*/
	for (int var = 0; var <= 100; ++var) {

		if (GNSS->uartWorkingBuffer[var] == 0xB5
				&& GNSS->uartWorkingBuffer[var + 1] == 0x62) {
			if (GNSS->uartWorkingBuffer[var + 2] == 0x27
					&& GNSS->uartWorkingBuffer[var + 3] == 0x03) { //Look at: 32.19.1.1 u-blox 8 Receiver description
				GNSS_ParseUniqID(GNSS);
			} else if (GNSS->uartWorkingBuffer[var + 2] == 0x01
					&& GNSS->uartWorkingBuffer[var + 3] == 0x21) { //Look at: 32.17.14.1 u-blox 8 Receiver description
				GNSS_ParseNavigatorData(GNSS);
			} else if (GNSS->uartWorkingBuffer[var + 2] == 0x01
					&& GNSS->uartWorkingBuffer[var + 3] == 0x07) { //ook at: 32.17.30.1 u-blox 8 Receiver description
				GNSS_ParsePVTData(GNSS);
			} else if (GNSS->uartWorkingBuffer[var + 2] == 0x01
					&& GNSS->uartWorkingBuffer[var + 3] == 0x02) { // Look at: 32.17.15.1 u-blox 8 Receiver description
				GNSS_ParsePOSLLHData(GNSS);
			} else if (GNSS->uartWorkingBuffer[var + 2] == 0x01
				    && GNSS->uartWorkingBuffer[var + 3] == 0x35) {  //Look at: 32.17.20.1 u-blox 8 Receiver description
				GNSS_ParseNAVSATData(GNSS);
			}
			/*else if (GNSS->uartWorkingBuffer[var + 2] == 0x05
				    && GNSS->uartWorkingBuffer[var + 3] == 0x01) {
				//printf("ACK \r\n");
			} else if (GNSS->uartWorkingBuffer[var + 2] == 0x05
				    && GNSS->uartWorkingBuffer[var + 3] == 0x00) {
				printf("NACK \r\n");
			}*/
		}
	}
}

/*!
 * Make request for unique chip ID data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetUniqID(GNSS_StateHandle *GNSS) {
	HAL_UART_Transmit_DMA(GNSS->huart, getDeviceID, sizeof(getDeviceID) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 17);
	/*printf("Device ID: ");
	for (uint8_t i = 0; i < 17; i++){
		printf("0x%X ", GNSS->uartWorkingBuffer[i]);
	}
	printf("\r\n");
	printf("\r\n");*/
}

/*!
 * Make request for UTC time solution data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetNavigatorData(GNSS_StateHandle *GNSS) {
	HAL_UART_Transmit_DMA(GNSS->huart, getNavigatorData, sizeof(getNavigatorData) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 28);
	/*printf("NAVIGATOR DATA: ");
	for (uint8_t i = 0; i < 28; i++){
			printf("0x%X ", GNSS->uartWorkingBuffer[i]);
		}
		printf("\r\n");
		printf("\r\n");*/
}

/*!
 * Make request for geodetic position solution data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetPOSLLHData(GNSS_StateHandle *GNSS) {
	HAL_UART_Transmit_DMA(GNSS->huart, getPOSLLHData, sizeof(getPOSLLHData) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 36);
	/*printf("POSLLH DATA: ");
	for (uint8_t i = 0; i < 36; i++){
			printf("0x%X ", GNSS->uartWorkingBuffer[i]);
		}
		printf("\r\n");
		printf("\r\n");*/
}

/*!
 * Make request for navigation position velocity time solution data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetPVTData(GNSS_StateHandle *GNSS) {
	HAL_UART_Transmit_DMA(GNSS->huart, getPVTData, sizeof(getPVTData) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 100);
	/*printf("PVT DATA: ");
	for (uint8_t i = 0; i < 100; i++){
			printf("0x%X ", GNSS->uartWorkingBuffer[i]);
		}
		printf("\r\n");
		printf("\r\n");*/
}

void GNSS_GetDOPData(GNSS_StateHandle *GNSS){
	HAL_UART_Transmit_DMA(GNSS->huart, getPVTData, sizeof(getPVTData) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 26);
}


/*!
 * Make request for satellite information data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetNAVSATData(GNSS_StateHandle *GNSS) {
	HAL_UART_Transmit_DMA(GNSS->huart, getNAVSATData, sizeof(getNAVSATData) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 28);
}

/*!
 * Parse data to unique chip ID standard.
 * Look at: 32.19.1.1 u-blox 8 Receiver description
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParseUniqID(GNSS_StateHandle *GNSS) {
	for (int var = 0; var < 5; ++var) {
		GNSS->uniqueID[var] = GNSS_Handle.uartWorkingBuffer[10 + var];
	}
	/*printf("uniqueID: ");
	for (uint8_t i = 0; i < 5; i++){
			printf("0x%X ", GNSS->uniqueID[i]);
		}
		printf("\r\n");
		printf("\r\n");*/
}

/*!
 * Changing the GNSS mode.
 * Look at: 32.10.19 u-blox 8 Receiver description
 */
void GNSS_SetMode(GNSS_StateHandle *GNSS, short gnssMode) {
	if (gnssMode == 0) {
		HAL_UART_Transmit_DMA(GNSS->huart, setPortableMode,sizeof(setPortableMode) / sizeof(uint8_t));
	} else if (gnssMode == 1) {
		HAL_UART_Transmit_DMA(GNSS->huart, setStationaryMode,sizeof(setStationaryMode) / sizeof(uint8_t));
	} else if (gnssMode == 2) {
		HAL_UART_Transmit_DMA(GNSS->huart, setPedestrianMode,sizeof(setPedestrianMode) / sizeof(uint8_t));
	} else if (gnssMode == 3) {
		HAL_UART_Transmit_DMA(GNSS->huart, setAutomotiveMode,sizeof(setAutomotiveMode) / sizeof(uint8_t));
	} else if (gnssMode == 4) {
		HAL_UART_Transmit_DMA(GNSS->huart, setAutomotiveMode,sizeof(setAutomotiveMode) / sizeof(uint8_t));
	} else if (gnssMode == 5) {
		HAL_UART_Transmit_DMA(GNSS->huart, setAirbone1GMode,sizeof(setAirbone1GMode) / sizeof(uint8_t));
	} else if (gnssMode == 6) {
		HAL_UART_Transmit_DMA(GNSS->huart, setAirbone2GMode,sizeof(setAirbone2GMode) / sizeof(uint8_t));
	} else if (gnssMode == 7) {
		HAL_UART_Transmit_DMA(GNSS->huart, setAirbone4GMode,sizeof(setAirbone4GMode) / sizeof(uint8_t));
	} else if (gnssMode == 8) {
		HAL_UART_Transmit_DMA(GNSS->huart, setWirstMode,sizeof(setWirstMode) / sizeof(uint8_t));
	} else if (gnssMode == 9) {
		HAL_UART_Transmit_DMA(GNSS->huart, setBikeMode,sizeof(setBikeMode) / sizeof(uint8_t));
	}
}
/*!
 * Parse data to navigation position velocity time solution standard.
 * Look at: 32.17.15.1 u-blox 8 Receiver description.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParsePVTData(GNSS_StateHandle *GNSS) {
	uShort.bytes[0] = GNSS_Handle.uartWorkingBuffer[10];
	GNSS->yearBytes[0]=GNSS_Handle.uartWorkingBuffer[10];
	uShort.bytes[1] = GNSS_Handle.uartWorkingBuffer[11];
	GNSS->yearBytes[1]=GNSS_Handle.uartWorkingBuffer[11];
	GNSS->year = uShort.uShort;
	GNSS->month = GNSS_Handle.uartWorkingBuffer[12];
	GNSS->day = GNSS_Handle.uartWorkingBuffer[13];
	GNSS->hour = GNSS_Handle.uartWorkingBuffer[14];
	GNSS->min = GNSS_Handle.uartWorkingBuffer[15];
	GNSS->sec = GNSS_Handle.uartWorkingBuffer[16];
	GNSS->timeValid = GNSS_Handle.uartWorkingBuffer[17];

	GNSS->fixType = GNSS_Handle.uartWorkingBuffer[26];
	//new - numSV - Number of satellites used in Nav Solution
	GNSS->numSV = GNSS_Handle.uartWorkingBuffer[29];

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 30];
		GNSS->lonBytes[var]= GNSS_Handle.uartWorkingBuffer[var + 30];
	}
	GNSS->lon = iLong.iLong;
	GNSS->fLon=(float)iLong.iLong/10000000.0;
	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 34];
		GNSS->latBytes[var]=GNSS_Handle.uartWorkingBuffer[var + 34];
	}
	GNSS->lat = iLong.iLong;
	GNSS->fLat=(float)iLong.iLong/10000000.0;
	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 38];
	}
	GNSS->height = iLong.iLong;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 42];
		GNSS->hMSLBytes[var] = GNSS_Handle.uartWorkingBuffer[var + 42];
	}
	GNSS->hMSL = iLong.iLong;

	for (int var = 0; var < 4; ++var) {
		uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 46];
	}
	GNSS->hAcc = (float)uLong.uLong/1000.0;

	for (int var = 0; var < 4; ++var) {
		uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 50];
	}
	GNSS->vAcc = uLong.uLong;

	//new velN - NED north velocity
	for (uint8_t var = 0; var < 4; ++var){
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[54+var];
	}
	GNSS->velN = (float)iLong.iLong/1000.0;

	//new velE - NED east velocity
	for (uint8_t var = 0; var < 4; ++var){
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[58+var];
	}
	GNSS->velE = (float)iLong.iLong/1000.0;

	//new velD - NED down velocity
	for (uint8_t var = 0; var < 4; ++var){
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[62+var];
	}
	GNSS->velD = (float)iLong.iLong/1000.0;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 66];
		GNSS->gSpeedBytes[var] = GNSS_Handle.uartWorkingBuffer[var + 66];
	}
	GNSS->gSpeed = (float)iLong.iLong/1000.0;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 70];
	}
	GNSS->headMot = iLong.iLong;
	GNSS->course = (float)GNSS->headMot* 1e-5;

	//new sAcc - Speed accuracy estimate
	for (uint8_t var = 0; var < 4; ++var){
		uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[74 + var];
	}
	GNSS->sAcc = (float)uLong.uLong/1000.0;

	//new headAcc - Speed accuracy estimate
	for (uint8_t var = 0; var < 4; ++var){
		uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[78 + var];
	}
	GNSS->headAcc = (float)uLong.uLong*1e-5;
}

void GNSS_ParseDOPData(GNSS_StateHandle *GNSS){
	for (uint8_t var = 0; var < 2; ++var){
		uShort.bytes[var] = GNSS_Handle.uartWorkingBuffer[18 + var];
	}
	GNSS->hDop = (float)uShort.uShort*0.01;

	for (uint8_t var = 0; var < 2; ++var){
		uShort.bytes[var] = GNSS_Handle.uartWorkingBuffer[20 + var];
	}
	GNSS->nDop = (float)uShort.uShort*0.01;

	for (uint8_t var = 0; var < 2; ++var){
		uShort.bytes[var] = GNSS_Handle.uartWorkingBuffer[22 + var];
	}
	GNSS->nDop = (float)uShort.uShort*0.01;
}

/*!
 * Parse data to UTC time solution standard.
 * Look at: 32.17.30.1 u-blox 8 Receiver description.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParseNavigatorData(GNSS_StateHandle *GNSS) {
	uShort.bytes[0] = GNSS_Handle.uartWorkingBuffer[18];
	uShort.bytes[1] = GNSS_Handle.uartWorkingBuffer[19];
	GNSS->year = uShort.uShort;
	GNSS->month = GNSS_Handle.uartWorkingBuffer[20];
	GNSS->day = GNSS_Handle.uartWorkingBuffer[21];
	GNSS->hour = GNSS_Handle.uartWorkingBuffer[22];
	GNSS->min = GNSS_Handle.uartWorkingBuffer[23];
	GNSS->sec = GNSS_Handle.uartWorkingBuffer[24];
}

void GNSS_ParseNAVSATData(GNSS_StateHandle *GNSS) {
	GNSS->satCount = GNSS_Handle.uartWorkingBuffer[11];
}

/*!
 * Parse data to geodetic position solution standard.
 * Look at: 32.17.14.1 u-blox 8 Receiver description.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParsePOSLLHData(GNSS_StateHandle *GNSS) {
	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 10];
	}
	GNSS->lon = iLong.iLong;
	GNSS->fLon=(float)iLong.iLong/10000000.0;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 14];
	}
	GNSS->lat = iLong.iLong;
	GNSS->fLat=(float)iLong.iLong/10000000.0;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 18];
	}
	GNSS->height = iLong.iLong;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 22];
	}
	GNSS->hMSL = iLong.iLong;

	for (int var = 0; var < 4; ++var) {
		uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 26];
	}
	GNSS->hAcc = uLong.uLong;

	for (int var = 0; var < 4; ++var) {
		uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 30];
	}
	GNSS->vAcc = uLong.uLong;
}

/*!
 *  Sends the basic configuration: Activation of the UBX standard, change of NMEA version to 4.10 and turn on of the Galileo system.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_LoadConfig(GNSS_StateHandle *GNSS) {
	HAL_UART_Transmit_DMA(GNSS->huart, configUBX, sizeof(configUBX) / sizeof(uint8_t));
	//osDelay(250);
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 10);
	osDelay(150);
	/*printf("Config UBX: ");
	for( uint8_t i = 0; i < 9; i++){
		printf("0x%X ", GNSS->uartWorkingBuffer[i]);
	}
	printf("\r\n");
	printf("\r\n");*/
	HAL_UART_Transmit_DMA(GNSS->huart, setNMEA410, sizeof(setNMEA410) / sizeof(uint8_t));

	//osDelay(250);
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 10);
	osDelay(150);
	/*printf("setNMEA: ");
		for( uint8_t i = 0; i < 9; i++){
			printf("0x%X ", GNSS->uartWorkingBuffer[i]);
		}
		printf("\r\n");
		printf("\r\n");*/
	HAL_UART_Transmit_DMA(GNSS->huart, setGNSS, sizeof(setGNSS) / sizeof(uint8_t));
	//osDelay(250);
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 10);
	osDelay(150);
	/*printf("setGNSS: ");
			for( uint8_t i = 0; i < 9; i++){
				printf("0x%X ", GNSS->uartWorkingBuffer[i]);
			}
			printf("\r\n");
			printf("\r\n");*/
}



/*!
 *  Creates a checksum based on UBX standard.
 * @param class Class value from UBX doc.
 * @param messageID MessageID value from UBX doc.
 * @param dataLength Data length value from UBX doc.
 * @param payload Just payload.
 * @return  Returns checksum.
 */
uint8_t GNSS_Checksum(uint8_t class, uint8_t messageID, uint8_t dataLength,uint8_t *payload) {
//todo: Look at 32.4 UBX Checksum
	return 0;
}
