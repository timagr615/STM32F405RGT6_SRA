/*
 * lsm6ds3.h
 *
 *  Created on: 1 февр. 2023 г.
 *      Author: tima
 */

#ifndef INC_LSM6DS3_H_
#define INC_LSM6DS3_H_

#include "stm32f4xx_hal.h"
#include "main.h"

#define LSM6DS3_DEFAULT_TIMEOUT 				100

#define LSM6DS3_SA0_LOW_LEVEL 					0
#define LSM6DS3_I2C_ADDRESS_0 					(0x6A << 1)

#define LSM6DS3_SA0_HIGH_LEVEL 					1
#define LSM6DS3_I2C_ADDRESS_1 					(0x6B << 1)

#define LSM6DS3_UNITS_G 						0
#define LSM6DS3_UNITS_MG 						1

/************** Register description *******************/
#define LSM6DS3_FUNC_CFG_ACCESS  				0x01
#define LSM6DS3_XL_GS_SENSOR_SYNC_TIME  		0x04
#define LSM6DS3_XL_GS_SENSOR_SYNC_EN  			0x05
#define LSM6DS3_XL_GS_FIFO_CTRL1  				0x06
#define LSM6DS3_XL_GS_FIFO_CTRL2  				0x07
#define LSM6DS3_XL_GS_FIFO_CTRL3  				0x08
#define LSM6DS3_XL_GS_FIFO_CTRL4  				0x09
#define LSM6DS3_XL_GS_FIFO_CTRL5  				0x0A
#define LSM6DS3_XL_GS_ORIENT_CFG_G  			0x0B
#define LSM6DS3_XL_GS_REFERENCE_G  				0x0C
#define LSM6DS3_XL_GS_INT1_CTRL  				0x0D
#define LSM6DS3_XL_GS_INT2_CTRL  				0x0E
#define LSM6DS3_XL_GS_WHO_AM_I_REG  			0x0F
#define LSM6DS3_XL_GS_CTRL1_XL  				0x10
#define LSM6DS3_XL_GS_CTRL2_G  					0x11
#define LSM6DS3_XL_GS_CTRL3_C  					0x12
#define LSM6DS3_XL_GS_CTRL4_C  					0x13
#define LSM6DS3_XL_GS_CTRL5_C  					0x14
#define LSM6DS3_XL_GS_CTRL6_G  					0x15
#define LSM6DS3_XL_GS_CTRL7_G  					0x16
#define LSM6DS3_XL_GS_CTRL8_XL  				0x17
#define LSM6DS3_XL_GS_CTRL9_XL  				0x18
#define LSM6DS3_XL_GS_CTRL10_C  				0x19
#define LSM6DS3_XL_GS_MASTER_CONFIG  			0x1A
#define LSM6DS3_XL_GS_WAKE_UP_SRC  				0x1B
#define LSM6DS3_XL_GS_TAP_SRC  					0x1C
#define LSM6DS3_XL_GS_D6D_SRC  					0x1D
#define LSM6DS3_XL_GS_STATUS_REG  				0x1E
#define LSM6DS3_XL_GS_OUT_TEMP_L  				0x20
#define LSM6DS3_XL_GS_OUT_TEMP_H  				0x21
#define LSM6DS3_XL_GS_OUTX_L_GS  				0x22
#define LSM6DS3_XL_GS_OUTX_H_GS  				0x23
#define LSM6DS3_XL_GS_OUTY_L_GS  				0x24
#define LSM6DS3_XL_GS_OUTY_H_GS  				0x25
#define LSM6DS3_XL_GS_OUTZ_L_GS  				0x26
#define LSM6DS3_XL_GS_OUTZ_H_GS  				0x27
#define LSM6DS3_XL_GS_OUTX_L_XL  				0x28
#define LSM6DS3_XL_GS_OUTX_H_XL  				0x29
#define LSM6DS3_XL_GS_OUTY_L_XL  				0x2A
#define LSM6DS3_XL_GS_OUTY_H_XL  				0x2B
#define LSM6DS3_XL_GS_OUTZ_L_XL  				0x2C
#define LSM6DS3_XL_GS_OUTZ_H_XL  				0x2D
#define LSM6DS3_XL_GS_SENSORHUB1_REG  			0x2E
#define LSM6DS3_XL_GS_SENSORHUB2_REG  			0x2F
#define LSM6DS3_XL_GS_SENSORHUB3_REG  			0x30
#define LSM6DS3_XL_GS_SENSORHUB4_REG  			0x31
#define LSM6DS3_XL_GS_SENSORHUB5_REG  			0x32
#define LSM6DS3_XL_GS_SENSORHUB6_REG  			0x33
#define LSM6DS3_XL_GS_SENSORHUB7_REG  			0x34
#define LSM6DS3_XL_GS_SENSORHUB8_REG  			0x35
#define LSM6DS3_XL_GS_SENSORHUB9_REG  			0x36
#define LSM6DS3_XL_GS_SENSORHUB10_REG  			0x37
#define LSM6DS3_XL_GS_SENSORHUB11_REG  			0x38
#define LSM6DS3_XL_GS_SENSORHUB12_REG  			0x39
#define LSM6DS3_XL_GS_FIFO_STATUS1  			0x3A
#define LSM6DS3_XL_GS_FIFO_STATUS2  			0x3B
#define LSM6DS3_XL_GS_FIFO_STATUS3  			0x3C
#define LSM6DS3_XL_GS_FIFO_STATUS4  			0x3D
#define LSM6DS3_XL_GS_FIFO_DATA_OUT_L  			0x3E
#define LSM6DS3_XL_GS_FIFO_DATA_OUT_H  			0x3F
#define LSM6DS3_XL_GS_TIMESTAMP0_REG  			0x40
#define LSM6DS3_XL_GS_TIMESTAMP1_REG  			0x41
#define LSM6DS3_XL_GS_TIMESTAMP2_REG  			0x42
#define LSM6DS3_XL_GS_STEP_COUNTER_L  			0x4B
#define LSM6DS3_XL_GS_STEP_COUNTER_H  			0x4C
#define LSM6DS3_XL_GS_FUNC_SRC  				0x53
#define LSM6DS3_XL_GS_TAP_CFG1  				0x58
#define LSM6DS3_XL_GS_TAP_THS_6D  				0x59
#define LSM6DS3_XL_GS_INT_DUR2  				0x5A
#define LSM6DS3_XL_GS_WAKE_UP_THS  				0x5B
#define LSM6DS3_XL_GS_WAKE_UP_DUR  				0x5C
#define LSM6DS3_XL_GS_FREE_FALL  				0x5D
#define LSM6DS3_XL_GS_MD1_CFG  					0x5E
#define LSM6DS3_XL_GS_MD2_CFG  					0x5F


#define LSM6DS3_WHO_I_AM_RESPONSE               0x69
#define LSM6DS3_DEFAULT_EFFORTS                 0x0A


/* Parameters related with Accelerometer's Initialization */
#define XL_POWER_DOWN                           0x00
#define XL_12_5Hz                               0x01
#define XL_26Hz                                 0x02
#define XL_52Hz                                 0x03
#define XL_104Hz                                0x04
#define XL_208Hz                                0x05
#define XL_416Hz                                0x06
#define XL_833Hz                                0x07
#define XL_1666Hz                               0x08
#define XL_3330Hz                               0x09
#define XL_6660Hz                               0x0A

#define XL_RANGE_2G                             0x00
#define XL_RANGE_16G                            0x01
#define XL_RANGE_4G                             0x02
#define XL_RANGE_8G                             0x03

#define XL_FILTER_400Hz                         0x00
#define XL_FILTER_200Hz                         0x01
#define XL_FILTER_100Hz                         0x02
#define XL_FILTER_50Hz                          0x03

/* Parameters related with Gyroscope's Initialization */
#define GS_POWER_DOWN                           0x00
#define GS_12_5Hz                               0x01
#define GS_26Hz                                 0x02
#define GS_52Hz                                 0x03
#define GS_104Hz                                0x04
#define GS_208Hz                                0x05
#define GS_416Hz                                0x06
#define GS_833Hz                                0x07
#define GS_1666Hz                               0x08

#define GS_RANGE_125dps                         0x01
#define GS_RANGE_250dps                         0x00
#define GS_RANGE_500dps                         0x02
#define GS_RANGE_1000dps                        0x04
#define GS_RANGE_2000dps                        0x06

#define GS_DRDY_INT1 							0x02

//BIASES ACC: -0.0012 0.0068 0.0163 GYRO: 0.0382 -0.6991 -1.9551
//BIASES ACC: -0.0020 0.0067 0.0136 GYRO: 1.4438 -2.7055 -1.6650
//BIASES ACC: -0.0019 0.0045 0.0146 GYRO: 1.3443 -2.4672 -1.6217

//BIASES ACC: -0.0057 -0.0018 1.0148 GYRO: 1.5391 -2.9414 -1.4465
//BIASES ACC: -0.0056 0.0111 1.0165 GYRO: 1.5783 -2.9704 -1.5568
//BIASES ACC: -0.0028 0.0111 1.0182 GYRO: 1.4871 -2.9186 -1.3141
//BIASES ACC: -0.0023 0.0111 1.0176 GYRO: 1.5715 -2.9340 -1.4043
#define AX_BIAS -0.0036
#define AY_BIAS 0.0111
#define AZ_BIAS 0.0174
#define GX_BIAS 1.5456
#define GY_BIAS -2.941
#define GZ_BIAS -1.425

typedef struct {
	uint8_t XL_range;
	uint8_t GS_range;
	uint16_t adrs;
	uint16_t timeout;
	I2C_HandleTypeDef* i2c_channel;
} LSM6DS3_configuration;


typedef struct {
	int16_t XL_x;
	int16_t XL_y;
	int16_t XL_z;
	int16_t GS_x;
	int16_t GS_y;
	int16_t GS_z;
	int16_t Temperature;
} LSM6DS3_IMU_measurements;

typedef struct {
	float a[3];
	float g[3];

	float a_bias[3];
	float g_bias[3];
} LSM6DS3_Data_Bias;

typedef enum {
	LSM6DS3_OK		= 		0,
	LSM6DS3_FAIL	= 		1,
	LSM6DS3_FAIL_PARAMETERS = 2,
	LSM6DS3_FAIL_COMPARE = 3,
} LSM6DS3_Result;


LSM6DS3_configuration LSM6DS3_parameters;
LSM6DS3_IMU_measurements LSM6DS3_data;
LSM6DS3_Data_Bias LSM6DS3;

void LSM6DS3_Init(uint8_t SA0, uint16_t timeout, I2C_HandleTypeDef* i2c_channel);
LSM6DS3_Result LSM6DS3_Configure(uint8_t efforts, uint8_t XL_Rate, uint8_t XL_Scale, uint8_t XL_Filter, uint8_t GS_Rate, uint8_t GS_Range);
LSM6DS3_Result LSM6DS3_Detect(uint8_t efforts);
LSM6DS3_Result LSM6DS3_INT1_Set_G_DRDY(void);
LSM6DS3_Result LSM6DS3_XL_Start(uint8_t XL_Rate, uint8_t XL_Scale, uint8_t XL_Filter);
LSM6DS3_Result LSM6DS3_XL_Stop(void);
LSM6DS3_Result LSM6DS3_XL_GetMesurements(void);
LSM6DS3_Result LSM6DS3_GS_Start(uint8_t GS_Rate, uint8_t GS_Range);
LSM6DS3_Result LSM6DS3_GS_Stop(void);
LSM6DS3_Result LSM6DS3_GS_GetMeasurements(void);
LSM6DS3_Result LSM6DS3_IMU_GetMeasurements(void);
LSM6DS3_Result LSM6DS3_IMU_GetData(void);
int16_t LSM6DS3_GetXL_X_Int16(void);
int16_t LSM6DS3_GetXL_Y_Int16(void);
int16_t LSM6DS3_GetXL_Z_Int16(void);
float LSM6DS3_GetXL_X_Float(uint8_t units);
float LSM6DS3_GetXL_Y_Float(uint8_t units);
float LSM6DS3_GetXL_Z_Float(uint8_t units);
int16_t LSM6DS3_GetGS_X_Int16(void);
int16_t LSM6DS3_GetGS_Y_Int16(void);
int16_t LSM6DS3_GetGS_Z_Int16(void);
float LSM6DS3_GetGS_X_Float(uint8_t units);
float LSM6DS3_GetGS_Y_Float(uint8_t units);
float LSM6DS3_GetGS_Z_Float(uint8_t units);
int16_t LSM6DS3_Temperature_Int16(void);
float LSM6DS3_Temperature_Celsius(void);


#endif /* INC_LSM6DS3_H_ */
