/*
 * lis3mdl.c
 *
 *  Created on: Feb 2, 2023
 *      Author: tima
 */

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdio.h>
#include "lis3mdl.h"
#include "main.h"

static LIS3MDL_Result LIS3MDL_Verify_XL_Settings(uint8_t XL_Rate, uint8_t XL_Range, uint8_t XL_Filter);
static LIS3MDL_Result LIS3MDL_Verify_GS_Settings(uint8_t GS_Rate, uint8_t GS_Range);
static LIS3MDL_Result LIS3MDL_WriteAndVerify(uint16_t reg, uint8_t* data, uint16_t size);
static LIS3MDL_Result LIS3MDL_ReadRegister(uint16_t reg, uint8_t* data, uint16_t size);




void LIS3MDL_Init(uint8_t SA1, uint16_t timeout, I2C_HandleTypeDef* i2c_channel){
	if(SA1){
		LIS3MDL_parameters.adrs = LIS3MDL_ADDRESS1;
		} else {
			LIS3MDL_parameters.adrs = LIS3MDL_ADDRESS2;
		}
	LIS3MDL_parameters.timeout = timeout;
	LIS3MDL_parameters.i2c_channel = i2c_channel;
	LIS3MDL.m[0] = 0.0;
	LIS3MDL.m[1] = 0.0;
	LIS3MDL.m[2] = 0.0;
}
LIS3MDL_Result LIS3MDL_Configure(uint8_t efforts, uint8_t M_rate, uint8_t M_scale){
	LIS3MDL_parameters.M_range = M_scale;
	if(LIS3MDL_Detect(efforts) == LIS3MDL_OK){
		uint8_t byte = LIS3MDL_CTRL_REG3_CONT_MODE;
		if(LIS3MDL_WriteAndVerify(LIS3MDL_CTRL_REG3, &byte, sizeof(byte)) != LIS3MDL_OK){
			printf("LIS3MDL FAIL CTRL REG 5 \r\n");
			return LIS3MDL_FAIL;
		}
		if(LIS3MDL_WriteAndVerify(LIS3MDL_CTRL_REG2, &M_scale, sizeof(M_scale)) != LIS3MDL_OK){
							printf("LIS3MDL FAIL CTRL REG 2 \r\n");
							return LIS3MDL_FAIL;
				}
		//osDelay(10);
		uint8_t reg1 = 0xfc;
		if(LIS3MDL_WriteAndVerify(LIS3MDL_CTRL_REG1, &M_rate, sizeof(M_rate)) != LIS3MDL_OK){
			printf("LIS3MDL FAIL CTRL REG 1 \r\n");
			return LIS3MDL_FAIL;
		}
		/*if(LIS3MDL_WriteAndVerify(LIS3MDL_CTRL_REG1, &reg1, sizeof(reg1)) != LIS3MDL_OK){
					printf("LIS3MDL FAIL CTRL REG 1 \r\n");
					return LIS3MDL_FAIL;
				}*/
		//osDelay(10);
		byte = LIS3MDL_CTRL_REG5_FAST;
		/*if(LIS3MDL_WriteAndVerify(LIS3MDL_CTRL_REG5, &byte, sizeof(byte)) != LIS3MDL_OK){
			printf("LIS3MDL FAIL CTRL REG 5 \r\n");
			return LIS3MDL_FAIL;
		}*/
		byte = 0b00000000;
		if(LIS3MDL_WriteAndVerify(LIS3MDL_CTRL_REG4, &byte, sizeof(byte)) != LIS3MDL_OK){
					printf("LIS3MDL FAIL CTRL REG 4 \r\n");
					return LIS3MDL_FAIL;
				}
		//osDelay(2);

	}
	else
	{
		printf("LIS DETECT FAIL \r\n");
		return LIS3MDL_FAIL;
	}
	if (DEBUG && DEBUG_LWL == 1){
		printf("LIS3MDL SUCCESS INIT \r\n");
	}
	return LIS3MDL_OK;
}
LIS3MDL_Result LIS3MDL_Detect(uint8_t efforts){
	uint8_t byte;

		while(efforts > 0){
			if(LIS3MDL_ReadRegister(LIS3MDL_WHO_AM_I, &byte, sizeof(byte)) == LIS3MDL_OK)
			{
				if(byte == LIS3MDL_WHO_AM_I_RESPONSE){
					return LIS3MDL_OK;
				}

			}
			osDelay(2);
			efforts--;
		}
		return LIS3MDL_FAIL;
}

void printBit(size_t const size, void const * const ptr)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;
    printf("Status bytes ");
    for (i = size-1; i >= 0; i--) {
        for (j = 7; j >= 0; j--) {
            byte = (b[i] >> j) & 1;
            printf("%u", byte);
        }
    }
    printf("\r\n");
}

LIS3MDL_Result LIS3MDL_ReadStatus(){
	uint8_t byte;
	if(LIS3MDL_ReadRegister(LIS3MDL_STATUS_REG, &byte, sizeof(byte)) == LIS3MDL_OK){
		printBit(1, &byte);
	}
}

LIS3MDL_Result LIS3MDL_MAG_GetMeasurements(void){
	uint8_t bytes[6] = {0};
	//LIS3MDL_ReadStatus();
		if(LIS3MDL_ReadRegister(LIS3MDL_OUT_X_L, bytes, sizeof(bytes)) == LIS3MDL_OK) {
			LIS3MDL_data.M_x 		 = ((int16_t) bytes[1]<<8)  | (int16_t)bytes[0];
			LIS3MDL_data.M_y		 = ((int16_t) bytes[3]<<8)  | (int16_t)bytes[2];
			LIS3MDL_data.M_z		 = ((int16_t) bytes[5]<<8)  | (int16_t)bytes[4];
			/*printf("data bytes: ");
			for(uint i = 0; i < 6; i++){
				printBit(1, &bytes[i]);
			}
			printf("\r\n");*/
			//printf("LIS DATA: %d %d %d \r\n", LIS3MDL_data.M_x, LIS3MDL_data.M_y, LIS3MDL_data.M_z);
			return LIS3MDL_OK;
		}
		return LIS3MDL_FAIL;
}

LIS3MDL_Result LIS3MDL_MAG_GetMeasurements_gauss(void){
	if(LIS3MDL_MAG_GetMeasurements() != LIS3MDL_OK){
		printf("LIS3MDL READ DATA FAIL \r\n");
		return LIS3MDL_FAIL;
	}
	//printf("LIS3MDL READ DATA OK \r\n");
	switch(LIS3MDL_parameters.M_range)
		{
			case LIS3MDL_CTRL_REG2_4G:
				LIS3MDL_data.Mg_x = (float)(LIS3MDL_data.M_x/6842.0f);
				LIS3MDL_data.Mg_y = (float)(LIS3MDL_data.M_y/6842.0f);
				LIS3MDL_data.Mg_z = (float)(LIS3MDL_data.M_z/6842.0f);
				break;
			case LIS3MDL_CTRL_REG2_8G:
				LIS3MDL_data.Mg_x = (float)(LIS3MDL_data.M_x/3421.0f);
				LIS3MDL_data.Mg_y = (float)(LIS3MDL_data.M_y/3421.0f);
				LIS3MDL_data.Mg_z = (float)(LIS3MDL_data.M_z/3421.0f);
				break;
			case LIS3MDL_CTRL_REG2_12G:
				LIS3MDL_data.Mg_x = (float)(LIS3MDL_data.M_x/2281.0f);
				LIS3MDL_data.Mg_y = (float)(LIS3MDL_data.M_y/2281.0f);
				LIS3MDL_data.Mg_z = (float)(LIS3MDL_data.M_z/2281.0f);

				break;
			case LIS3MDL_CTRL_REG2_16G:
				LIS3MDL_data.Mg_x = ((float)LIS3MDL_data.M_x)/1711.0f;
				LIS3MDL_data.Mg_y = ((float)LIS3MDL_data.M_y)/1711.0f;
				LIS3MDL_data.Mg_z = ((float)LIS3MDL_data.M_z)/1711.0f;
				//printf("M range : 0x%X %d %d %d \r\n", LIS3MDL_parameters.M_range, LIS3MDL_data.M_x, LIS3MDL_data.M_y, LIS3MDL_data.M_z);
				//printf("M gauss : %.2f %.2f %.2f \r\n", LIS3MDL_data.Mg_x, LIS3MDL_data.Mg_y, LIS3MDL_data.Mg_z);
				break;
			default:
				LIS3MDL_data.Mg_x = 0.0;
				LIS3MDL_data.Mg_y = 0.0;
				LIS3MDL_data.Mg_z = 0.0;
				break;
		}
	LIS3MDL.m[0] = (LIS3MDL_data.Mg_x - MAG_BIAS_X)*MAG_SCALE_X;
	LIS3MDL.m[1] = (LIS3MDL_data.Mg_y - MAG_BIAS_Y)*MAG_SCALE_Y;
	LIS3MDL.m[2] = (LIS3MDL_data.Mg_z - MAG_BIAS_Z)*MAG_SCALE_Z;
	/*LIS3MDL.m[0] = LIS3MDL_data.Mg_x;
		LIS3MDL.m[1] = LIS3MDL_data.Mg_y;
		LIS3MDL.m[2] = LIS3MDL_data.Mg_z;*/
	return LIS3MDL_OK;
}

static LIS3MDL_Result LIS3MDL_ReadRegister(uint16_t reg, uint8_t* data, uint16_t size){
	if(HAL_I2C_Mem_Read(LIS3MDL_parameters.i2c_channel, LIS3MDL_parameters.adrs, (0x00FF & reg),
			I2C_MEMADD_SIZE_8BIT, data, size, LIS3MDL_parameters.timeout) != HAL_OK) {
		return LIS3MDL_FAIL;
	}
	return LIS3MDL_OK;
}

static LIS3MDL_Result LIS3MDL_WriteAndVerify(uint16_t reg, uint8_t* data, uint16_t size){
	uint8_t compare[32] = {0xFF};

	if(size > sizeof(compare)){
		return LIS3MDL_FAIL_PARAMETERS;
	}
	if(HAL_I2C_Mem_Write(LIS3MDL_parameters.i2c_channel, LIS3MDL_parameters.adrs, (0x00FF & reg), I2C_MEMADD_SIZE_8BIT, data, size, LIS3MDL_parameters.timeout) != HAL_OK)
			return LIS3MDL_FAIL;
	if(HAL_I2C_Mem_Read(LIS3MDL_parameters.i2c_channel, LIS3MDL_parameters.adrs, (0x00FF & reg), I2C_MEMADD_SIZE_8BIT, compare, size, LIS3MDL_parameters.timeout) != HAL_OK)
		return LIS3MDL_FAIL;
	for(uint8_t i=0; i < size; i++) {
		if(compare[i] != data[i])
			return LIS3MDL_FAIL_COMPARE;
	}
	return LIS3MDL_OK;

}

