/*
 * lsm6ds3.c
 *
 *  Created on: 1 февр. 2023 г.
 *      Author: tima
 */
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "lsm6ds3.h"

static LSM6DS3_Result LSM6DS3_Verify_XL_Settings(uint8_t XL_Rate, uint8_t XL_Range, uint8_t XL_Filter);
static LSM6DS3_Result LSM6DS3_Verify_GS_Settings(uint8_t GS_Rate, uint8_t GS_Range);
static LSM6DS3_Result LSM6DS3_WriteAndVerify(uint16_t reg, uint8_t* data, uint16_t size);
static LSM6DS3_Result LSM6DS3_ReadRegister(uint16_t reg, uint8_t* data, uint16_t size);




void LSM6DS3_Init(uint8_t SA0, uint16_t timeout, I2C_HandleTypeDef* i2c_channel) {
	if(SA0){
		LSM6DS3_parameters.adrs = LSM6DS3_I2C_ADDRESS_1;
	} else {
		LSM6DS3_parameters.adrs = LSM6DS3_I2C_ADDRESS_0;
	}
	LSM6DS3_parameters.timeout = timeout;
	LSM6DS3_parameters.i2c_channel = i2c_channel;
	for (uint8_t i = 0; i < 3; i++){
		LSM6DS3.a[i] = 0.0;
		LSM6DS3.g[i] = 0.0;
		LSM6DS3.a_bias[i] = 0.0;
		LSM6DS3.g_bias[i] = 0.0;
	}
}

LSM6DS3_Result LSM6DS3_Configure(uint8_t efforts, uint8_t XL_Rate, uint8_t XL_Scale, uint8_t XL_Filter, uint8_t GS_Rate, uint8_t GS_Range){
	if(LSM6DS3_Detect(efforts) == LSM6DS3_OK) {
		LSM6DS3_XL_Start(XL_Rate, XL_Scale, XL_Filter);
		LSM6DS3_GS_Start(GS_Rate, GS_Range);
		osDelay(15);
		/*if(LSM6DS3_INT1_Set_G_DRDY() != LSM6DS3_OK){
			printf("LSM ERROR INT1!!!! \r\n");
			return LSM6DS3_FAIL;
		}*/

		if (DEBUG && DEBUG_LWL <= 1){
			printf("LSM_OK \r\n");
		}
		return LSM6DS3_OK;
	}
	else
	{
		printf("LSM ERROR!!!! \r\n");
		return LSM6DS3_FAIL;
	}
}

LSM6DS3_Result LSM6DS3_INT1_Set_G_DRDY(void){
	uint8_t byte = GS_DRDY_INT1;
		if(LSM6DS3_WriteAndVerify(LSM6DS3_XL_GS_INT2_CTRL, &byte, sizeof(byte)) == LSM6DS3_OK){
			return LSM6DS3_OK;
		}
		return LSM6DS3_FAIL;
}

LSM6DS3_Result LSM6DS3_Detect(uint8_t efforts){
	uint8_t byte;

	while(efforts > 0){
		printf("LSM DETECT 0x%X \r\n", byte);
		if(LSM6DS3_ReadRegister(LSM6DS3_XL_GS_WHO_AM_I_REG, &byte, sizeof(byte)) == LSM6DS3_OK)
		{

			if(byte == LSM6DS3_WHO_I_AM_RESPONSE){
				return LSM6DS3_OK;
			}

		}
		osDelay(2);
		efforts--;
	}
	return LSM6DS3_FAIL;
}

/*
 * @brief  Start the accelerometer with desired settings
 * @param  XL_Rate: Output data rate
 * 					- XL_12_5Hz
 *				    - XL_26Hz
 *					- XL_52Hz
 *					- XL_104Hz
 *					- XL_208Hz
 *					- XL_416Hz
 *					- XL_833Hz
 *					- XL_1666Hz
 *					- XL_3330Hz
 *					- XL_6660Hz
 *		   XL_Scale: Scale selection
 *		   			- XL_RANGE_2G
 *		   			- XL_RANGE_16G
 *		   			- XL_RANGE_4G
 *		   			- XL_RANGE_8G
 *         XL_Filter: Anti-aliasing filter bandwidth
 *					- XL_FILTER_400Hz
 * 					- XL_FILTER_200Hz
 * 					- XL_FILTER_100Hz
 * 					- XL_FILTER_50Hz
 * @retval enum LSM6DS3_Result:
 * 					- LSM6DS3_OK    		   = 0,  Success
 * 					- LSM6DS3_FAIL  		   = 1,  Failed due to HAL
 * 					- LSM6DS3_FAIL_PARAMETERS  = 2,  Failed due to wrong parameters
*/

LSM6DS3_Result LSM6DS3_XL_Start(uint8_t XL_Rate, uint8_t XL_Scale, uint8_t XL_Filter){
	uint8_t byte = (XL_Rate << 4) | ((XL_Scale << 2) & 0x0F) | (XL_Filter & 0x03);
	if (LSM6DS3_Verify_XL_Settings(XL_Rate, XL_Scale, XL_Filter) != LSM6DS3_OK){
		return LSM6DS3_FAIL_PARAMETERS;
	}
	if(LSM6DS3_WriteAndVerify(LSM6DS3_XL_GS_CTRL1_XL, &byte, sizeof(byte)) == LSM6DS3_OK){
		return LSM6DS3_OK;
	}
	return LSM6DS3_FAIL;
}

/*
 * @brief  Disable the accelerometer
 * @param  None
 * @retval enum LSM6DS3_Result:
 * 					- LSM6DS3_OK    		   = 0,  Success
 * 					- LSM6DS3_FAIL  		   = 1,  Failed due to HAL
*/

uint8_t LSM6DS3_XL_Stop(void) {
	uint8_t byte = XL_POWER_DOWN;

	if(LSM6DS3_WriteAndVerify(LSM6DS3_XL_GS_CTRL1_XL, &byte, sizeof(byte)) ==  LSM6DS3_OK)
		return LSM6DS3_OK;
	return LSM6DS3_FAIL;
}

/*
 * @brief  Check the Accelerometer settings
 * @param  None
 * @retval enum LSM6DS3_Result:
 * 					- LSM6DS3_OK    		   = 0,  Success
 * 					- LSM6DS3_FAIL_PARAMETERS  = 2,  Failed due to wrong parameters
*/
static LSM6DS3_Result LSM6DS3_Verify_XL_Settings(uint8_t XL_Rate, uint8_t XL_Range, uint8_t XL_Filter) {
	if((XL_Rate < XL_12_5Hz) || (XL_Rate > XL_6660Hz))
		return LSM6DS3_FAIL_PARAMETERS;
	else if((XL_Range < XL_RANGE_2G) || (XL_Range > XL_RANGE_8G))
		return LSM6DS3_FAIL_PARAMETERS;
	else if((XL_Filter < XL_FILTER_400Hz) || (XL_Filter > XL_FILTER_50Hz))
		return LSM6DS3_FAIL_PARAMETERS;
	LSM6DS3_parameters.XL_range = XL_Range;
	return LSM6DS3_OK;
}


/*
 * @brief  Refresh the Accelerometer data
 * @param  None
 * @retval enum LSM6DS3_Result:
 * 					- LSM6DS3_OK    		   = 0,  Success
 * 					- LSM6DS3_FAIL  		   = 1,  Failed due to HAL
*/
LSM6DS3_Result LSM6DS3_XL_GetMeasurements(void) {
	uint8_t bytes[6] = {0};

	if(LSM6DS3_ReadRegister(LSM6DS3_XL_GS_OUTX_L_XL, bytes, sizeof(bytes)) == LSM6DS3_OK) {
		LSM6DS3_data.XL_x = (int16_t) (bytes[1]<<8 | bytes[0]);
		LSM6DS3_data.XL_y = (int16_t) (bytes[3]<<8 | bytes[2]);
		LSM6DS3_data.XL_z = (int16_t) (bytes[5]<<8 | bytes[4]);
		return LSM6DS3_OK;
	}
	return LSM6DS3_FAIL;
}

/*
 * @brief  Start the gyroscope with desired settings
 * @param  GS_Rate: Output data rate
 * 					- GS_12_5Hz
 * 					- GS_26Hz
 * 					- GS_52Hz
 * 					- GS_104Hz
 * 					- GS_208Hz
 * 					- GS_416Hz
 * 					- GS_833Hz
 * 					- GS_1666Hz
 *		   GS_Range: Scale selection
 *		   			- GS_RANGE_125dps
 *		   			- GS_RANGE_250dps
 *		   			- GS_RANGE_500dps
 *		   			- GS_RANGE_1000dps
 *		   			- GS_RANGE_2000dps
 * @retval enum LSM6DS3_Result:
 * 					- LSM6DS3_OK    		   = 0,  Success
 * 					- LSM6DS3_FAIL  		   = 1,  Failed due to HAL
 * 					- LSM6DS3_FAIL_PARAMETERS  = 2,  Failed due to wrong parameters
*/
LSM6DS3_Result LSM6DS3_GS_Start(uint8_t GS_Rate, uint8_t GS_Range) {
	uint8_t byte = (GS_Rate << 4) | ((GS_Range << 1) & 0x0F);

	byte &= 0xFE; /* Always secure the LSB is 0 */
	if(LSM6DS3_Verify_GS_Settings(GS_Rate, GS_Range) !=  LSM6DS3_OK)
		return LSM6DS3_FAIL_PARAMETERS;
	if(LSM6DS3_WriteAndVerify(LSM6DS3_XL_GS_CTRL2_G, &byte, sizeof(byte)) ==  LSM6DS3_OK)
		return LSM6DS3_OK;
	return LSM6DS3_FAIL;
}

/*
 * @brief  Disable the gyroscope
 * @param  None
 * @retval enum LSM6DS3_Result:
 * 					- LSM6DS3_OK    		   = 0,  Success
 * 					- LSM6DS3_FAIL  		   = 1,  Failed due to HAL
*/
LSM6DS3_Result LSM6DS3_GS_Stop(void) {
	uint8_t byte = GS_POWER_DOWN;

	if(LSM6DS3_WriteAndVerify(LSM6DS3_XL_GS_CTRL2_G, &byte, sizeof(byte)) ==  LSM6DS3_OK)
		return LSM6DS3_OK;
	return LSM6DS3_FAIL;
}

/*
 * @brief  Check the Gyroscope settings
 * @param  None
 * @retval enum LSM6DS3_Result:
 * 					- LSM6DS3_OK    		   = 0,  Success
 * 					- LSM6DS3_FAIL_PARAMETERS  = 2,  Failed due to wrong parameters
*/
static LSM6DS3_Result LSM6DS3_Verify_GS_Settings(uint8_t GS_Rate, uint8_t GS_Range) {
	if((GS_Rate < GS_12_5Hz) || (GS_Rate > GS_1666Hz))
		return LSM6DS3_FAIL_PARAMETERS;
	else if((GS_Range < GS_RANGE_250dps) || (GS_Range > GS_RANGE_2000dps) || (GS_Range == 3) || (GS_Range == 5))
		return LSM6DS3_FAIL_PARAMETERS;
	LSM6DS3_parameters.GS_range = GS_Range;
	return LSM6DS3_OK;
}


/*
 * @brief  Refresh the Gyroscope data
 * @param  None
 * @retval enum LSM6DS3_Result:
* 					- LSM6DS3_OK    		   = 0,  Success
 * 					- LSM6DS3_FAIL  		   = 1,  Failed due to HAL
*/
LSM6DS3_Result LSM6DS3_GS_GetMeasurements(void) {
	uint8_t bytes[6] = {0};

	if(LSM6DS3_ReadRegister(LSM6DS3_XL_GS_OUTX_L_GS, bytes, sizeof(bytes)) == LSM6DS3_OK) {
		LSM6DS3_data.GS_x = (int16_t) (bytes[1]<<8 | bytes[0]);
		LSM6DS3_data.GS_y = (int16_t) (bytes[3]<<8 | bytes[2]);
		LSM6DS3_data.GS_z = (int16_t) (bytes[5]<<8 | bytes[4]);
		return LSM6DS3_OK;
	}
	return LSM6DS3_FAIL;
}


/*
 * @brief  Refresh the IMU values
 * @param  None
 * @retval enum LSM6DS3_Result:
* 					- LSM6DS3_OK    		   = 0,  Success
 * 					- LSM6DS3_FAIL  		   = 1,  Failed due to HAL
*/
LSM6DS3_Result LSM6DS3_IMU_GetMeasurements(void) {
	uint8_t bytes[14] = {0};

	if(LSM6DS3_ReadRegister(LSM6DS3_XL_GS_OUT_TEMP_L, bytes, sizeof(bytes)) == LSM6DS3_OK) {
		LSM6DS3_data.Temperature = (int16_t) (bytes[1]<<8  | bytes[0]);
		LSM6DS3_data.GS_x 		 = (int16_t) (bytes[3]<<8  | bytes[2]);
		LSM6DS3_data.GS_y 		 = (int16_t) (bytes[5]<<8  | bytes[4]);
		LSM6DS3_data.GS_z 		 = (int16_t) (bytes[7]<<8  | bytes[6]);
		LSM6DS3_data.XL_x 		 = (int16_t) (bytes[9]<<8  | bytes[8]);
		LSM6DS3_data.XL_y 		 = (int16_t) (bytes[11]<<8 | bytes[10]);
		LSM6DS3_data.XL_z 		 = (int16_t) (bytes[13]<<8 | bytes[12]);
		return LSM6DS3_OK;
	}
	return LSM6DS3_FAIL;
}


LSM6DS3_Result LSM6DS3_IMU_GetData(){
	if(LSM6DS3_IMU_GetMeasurements() != LSM6DS3_OK){
		return LSM6DS3_FAIL;
	}
	float mdps_lsb = 0;

		switch(LSM6DS3_parameters.GS_range)
		{
			case GS_RANGE_125dps:
				mdps_lsb = 4.375f;
				break;
			case GS_RANGE_250dps:
				mdps_lsb = 8.75f;
				break;
			case GS_RANGE_500dps:
				mdps_lsb = 17.50f;
				break;
			case GS_RANGE_1000dps:
				mdps_lsb = 35.0f;
				break;
			case GS_RANGE_2000dps:
				mdps_lsb = 70.f;
				break;
			default:
				mdps_lsb = 0.000f; /* Should never comes here */
				break;
		}

	float mg_lsb = 0;
	switch(LSM6DS3_parameters.XL_range)
	{
		case XL_RANGE_2G:
			mg_lsb = 0.061f;
			break;
		case XL_RANGE_4G:
			mg_lsb = 0.122f;
			break;
		case XL_RANGE_8G:
			mg_lsb = 0.244f;
			break;
		case XL_RANGE_16G:
			mg_lsb = 0.488f;
			break;
		default:
			mg_lsb = 0.000f; /* Should never comes here */
			break;
	}
	//printf("IN LSM FUNC %.2f \r\n", LSM6DS3_data.XL_x);
	LSM6DS3.a[0] = LSM6DS3_data.XL_x * mg_lsb / 1000.0f - AX_BIAS;
	LSM6DS3.a[1] = LSM6DS3_data.XL_y * mg_lsb / 1000.0f - AY_BIAS;
	LSM6DS3.a[2] = LSM6DS3_data.XL_z * mg_lsb / 1000.0f - AZ_BIAS;

	LSM6DS3.g[0] = LSM6DS3_data.GS_x * mdps_lsb / 1000.0f - GX_BIAS;
	LSM6DS3.g[1] = LSM6DS3_data.GS_y * mdps_lsb / 1000.0f - GY_BIAS;
	LSM6DS3.g[2] = LSM6DS3_data.GS_z * mdps_lsb / 1000.0f - GZ_BIAS;
	/*LSM6DS3.a[0] = LSM6DS3_data.XL_x * mg_lsb / 1000.0f;
		LSM6DS3.a[1] = LSM6DS3_data.XL_y * mg_lsb / 1000.0f;
		LSM6DS3.a[2] = LSM6DS3_data.XL_z * mg_lsb / 1000.0f;

		LSM6DS3.g[0] = LSM6DS3_data.GS_x * mdps_lsb / 1000.0f;
		LSM6DS3.g[1] = LSM6DS3_data.GS_y * mdps_lsb / 1000.0f;
		LSM6DS3.g[2] = LSM6DS3_data.GS_z * mdps_lsb / 1000.0f;*/
	return LSM6DS3_OK;
}


/*
 * @brief  Get the raw data of X axe (Accelerometer)
 * @param  none
 * @retval int16_t: Raw data of X axe
*/
int16_t LSM6DS3_GetXL_X_Int16(void) {
	return LSM6DS3_data.XL_x;
}

/*
 * @brief  Get the raw data of Y axe (Accelerometer)
 * @param  None
 * @retval int16_t: Raw data of Y axe
*/
int16_t LSM6DS3_GetXL_Y_Int16(void) {
	return LSM6DS3_data.XL_y;
}

/*
 * @brief  Get the raw data of Z axe (Accelerometer)
 * @param  None
 * @retval int16_t: Raw data of Z axe
*/
int16_t LSM6DS3_GetXL_Z_Int16(void) {
	return LSM6DS3_data.XL_z;
}

/*
 * @brief  Get the value of X axe (Accelerometer)
 * @param  units: Select the units to be returned
 * 					- 0:  Selected units are g
 * 					- 1:  Selected units are mg
 * 					- >1: Invalid. Will return zero
 * @retval float: Value of X axe on g or mg
*/
float LSM6DS3_GetXL_X_Float(uint8_t units) {
	float mg_lsb = 0;
	switch(LSM6DS3_parameters.XL_range)
	{
		case XL_RANGE_2G:
			mg_lsb = 0.061f;
			break;
		case XL_RANGE_4G:
			mg_lsb = 0.122f;
			break;
		case XL_RANGE_8G:
			mg_lsb = 0.244f;
			break;
		case XL_RANGE_16G:
			mg_lsb = 0.488f;
			break;
		default:
			mg_lsb = 0.000f; /* Should never comes here */
			break;
	}
	if(units == LSM6DS3_UNITS_G)
		return (LSM6DS3_data.XL_x * mg_lsb / 1000.0f);
	else
		return (LSM6DS3_data.XL_x * mg_lsb);
}

/*
 * @brief  Get the value of Y axe (Accelerometer)
 * @param  units: Select the units to be returned
 * 					- 0:  Selected units are g
 * 					- 1:  Selected units are mg
 * 					- >1: Invalid. Will return zero
 * @retval float: Value of Y axe on g or mg
*/
float LSM6DS3_GetXL_Y_Float(uint8_t units) {
	float mg_lsb = 0;

	switch(LSM6DS3_parameters.XL_range)
	{
		case XL_RANGE_2G:
			mg_lsb = 0.061f;
			break;
		case XL_RANGE_4G:
			mg_lsb = 0.122f;
			break;
		case XL_RANGE_8G:
			mg_lsb = 0.244f;
			break;
		case XL_RANGE_16G:
			mg_lsb = 0.488f;
			break;
		default:
			mg_lsb = 0.000f; /* Should never comes here */
			break;
	}
	if(units == LSM6DS3_UNITS_G)
		return (LSM6DS3_data.XL_y * mg_lsb / 1000.0f);
	else
		return (LSM6DS3_data.XL_y * mg_lsb);
}

/*
 * @brief  Get the value of Z axe (Accelerometer)
 * @param  units: Select the units to be returned
 * 					- 0:  Selected units are g
 * 					- 1:  Selected units are mg
 * 					- >1: Invalid. Will return zero
 * @retval float: Value of Z axe on g or mg
*/
float LSM6DS3_GetXL_Z_Float(uint8_t units) {
	float mg_lsb = 0;

	switch(LSM6DS3_parameters.XL_range)
	{
		case XL_RANGE_2G:
			mg_lsb = 0.061f;
			break;
		case XL_RANGE_4G:
			mg_lsb = 0.122f;
			break;
		case XL_RANGE_8G:
			mg_lsb = 0.244f;
			break;
		case XL_RANGE_16G:
			mg_lsb = 0.488f;
			break;
		default:
			mg_lsb = 0.000f; /* Should never comes here */
			break;
	}
	if(units == LSM6DS3_UNITS_G)
		return (LSM6DS3_data.XL_z * mg_lsb / 1000.0f);
	else
		return (LSM6DS3_data.XL_z * mg_lsb);
}

/*
 * @brief  Get the raw data of X axe (Gyroscope)
 * @param  None
 * @retval int16_t: Raw data of X axe
*/
int16_t LSM6DS3_GetGS_X_Int16(void) {
	return LSM6DS3_data.GS_x;
}

/*
 * @brief  Get the raw data of Y axe (Gyroscope)
 * @param  None
 * @retval int16_t: Raw data of Y axe
*/
int16_t LSM6DS3_GetGS_Y_Int16(void) {
	return LSM6DS3_data.GS_y;
}

/*
 * @brief  Get the raw data of Z axe (Gyroscope)
 * @param  None
 * @retval int16_t: Raw data of Z axe
*/
int16_t LSM6DS3_GetGS_Z_Int16(void) {
	return LSM6DS3_data.GS_z;
}

/*
 * @brief  Get the raw data of temperature
 * @param  None
 * @retval int16_t: Raw data of temperature
*/
int16_t LSM6DS3_Temperature_Int16(void) {
	return LSM6DS3_data.Temperature;
}

/*
 * @brief  Get the temperature
 * @param  None
 * @retval float: Temperature in C (Celsius)
*/
float LSM6DS3_Temperature_Celsius(void) {
	return ((float)LSM6DS3_data.Temperature / 16.0f + 25.0f);
}

/*
 * @brief  Get the value of X axe (Gyroscope)
 * @param  units: Select the units to be returned
 * 					- 0: Selected units are dps
 * 					- 1: Selected units are mdps
 * 					- >1: Invalid. Will return zero
 * @retval float: Value of X axe on dps or mdps
*/
float LSM6DS3_GetGS_X_Float(uint8_t units) {
	float mdps_lsb = 0;

	switch(LSM6DS3_parameters.GS_range)
	{
		case GS_RANGE_125dps:
			mdps_lsb = 4.375f;
			break;
		case GS_RANGE_250dps:
			mdps_lsb = 8.75f;
			break;
		case GS_RANGE_500dps:
			mdps_lsb = 17.50f;
			break;
		case GS_RANGE_1000dps:
			mdps_lsb = 35.0f;
			break;
		case GS_RANGE_2000dps:
			mdps_lsb = 70.f;
			break;
		default:
			mdps_lsb = 0.000f; /* Should never comes here */
			break;
	}
	if(units == LSM6DS3_UNITS_G)
		return (LSM6DS3_data.GS_x * mdps_lsb / 1000.0f);
	else
		return (LSM6DS3_data.GS_x * mdps_lsb);
}

/*
 * @brief  Get the value of Y axe (Gyroscope)
 * @param  units: Select the units to be returned
 * 					- 0: Selected units are dps
 * 					- 1: Selected units are mdps
 * 					- >1: Invalid. Will return zero
 * @retval float: Value of Y axe on dps or mdps
*/
float LSM6DS3_GetGS_Y_Float(uint8_t units) {
	float mdps_lsb = 0;

	switch(LSM6DS3_parameters.GS_range)
	{
		case GS_RANGE_125dps:
			mdps_lsb = 4.375f;
			break;
		case GS_RANGE_250dps:
			mdps_lsb = 8.75f;
			break;
		case GS_RANGE_500dps:
			mdps_lsb = 17.50f;
			break;
		case GS_RANGE_1000dps:
			mdps_lsb = 35.0f;
			break;
		case GS_RANGE_2000dps:
			mdps_lsb = 70.f;
			break;
		default:
			mdps_lsb = 0.000f; /* Should never comes here */
			break;
	}
	if(units == LSM6DS3_UNITS_G)
		return (LSM6DS3_data.GS_y * mdps_lsb / 1000.0f);
	else
		return (LSM6DS3_data.GS_y * mdps_lsb);
}

/*
 * @brief  Get the value of Z axe (Gyroscope)
 * @param  units: Select the units to be returned
 * 					- 0: Selected units are dps
 * 					- 1: Selected units are mdps
 * 					- >1: Invalid. Will return zero
 * @retval float: Value of Z axe on dps or mdps
*/
float LSM6DS3_GetGS_Z_Float(uint8_t units) {
	float mdps_lsb = 0;

	switch(LSM6DS3_parameters.GS_range)
	{
		case GS_RANGE_125dps:
			mdps_lsb = 4.375f;
			break;
		case GS_RANGE_250dps:
			mdps_lsb = 8.75f;
			break;
		case GS_RANGE_500dps:
			mdps_lsb = 17.50f;
			break;
		case GS_RANGE_1000dps:
			mdps_lsb = 35.0f;
			break;
		case GS_RANGE_2000dps:
			mdps_lsb = 70.f;
			break;
		default:
			mdps_lsb = 0.000f; /* Should never comes here */
			break;
	}
	if(units == LSM6DS3_UNITS_G)
		return (LSM6DS3_data.GS_z * mdps_lsb / 1000.0f);
	else
		return (LSM6DS3_data.GS_z * mdps_lsb);
}



static LSM6DS3_Result LSM6DS3_ReadRegister(uint16_t reg, uint8_t *data, uint16_t size){
	if(HAL_I2C_Mem_Read(LSM6DS3_parameters.i2c_channel, LSM6DS3_parameters.adrs, (0x00FF & reg),
			I2C_MEMADD_SIZE_8BIT, data, size, LSM6DS3_parameters.timeout) != HAL_OK) {
		if(reg == LSM6DS3_XL_GS_WHO_AM_I_REG){
			printf("DETECT FAIL IN READ REG 0x%X 0x%X \r\n", reg, *data);
		}
		return LSM6DS3_FAIL;
	}
	return LSM6DS3_OK;
}

static LSM6DS3_Result LSM6DS3_WriteAndVerify(uint16_t reg, uint8_t* data, uint16_t size){
	uint8_t compare[32] = {0xFF};

	if(size > sizeof(compare)){
		return LSM6DS3_FAIL_PARAMETERS;
	}
	if(HAL_I2C_Mem_Write(LSM6DS3_parameters.i2c_channel, LSM6DS3_parameters.adrs, (0x00FF & reg), I2C_MEMADD_SIZE_8BIT, data, size, LSM6DS3_parameters.timeout) != HAL_OK)
			return LSM6DS3_FAIL;
	if(HAL_I2C_Mem_Read(LSM6DS3_parameters.i2c_channel, LSM6DS3_parameters.adrs, (0x00FF & reg), I2C_MEMADD_SIZE_8BIT, compare, size, LSM6DS3_parameters.timeout) != HAL_OK)
		return LSM6DS3_FAIL;
	for(uint8_t i=0; i < size; i++) {
		if(compare[i] != data[i])
			return LSM6DS3_FAIL_COMPARE;
	}
	return LSM6DS3_OK;

}

