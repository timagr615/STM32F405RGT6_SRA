/*
 * lis3mdl.h
 *
 *  Created on: Feb 2, 2023
 *      Author: tima
 */

#ifndef INC_LIS3MDL_H_
#define INC_LIS3MDL_H_

#define LIS3MDL_SA1_LOW_LEVEL 					0
#define LIS3MDL_SA1_HIGH_LEVEL 					1
#define LIS3MDL_ADDRESS1  (0b0011110 << 1)
#define LIS3MDL_ADDRESS2  (0b0011100 << 1)

#define LIS3MDL_WHO_AM_I				0x0F

#define LIS3MDL_CTRL_REG1				0x20
#define LIS3MDL_CTRL_REG2				0x21
#define LIS3MDL_CTRL_REG3				0x22
#define LIS3MDL_CTRL_REG4				0x23
#define LIS3MDL_CTRL_REG5				0x24

#define LIS3MDL_STATUS_REG				0x27

#define LIS3MDL_OUT_X_L					0x28
#define LIS3MDL_OUT_X_H					0x29
#define LIS3MDL_OUT_Y_L					0x2A
#define LIS3MDL_OUT_Y_H					0x2B
#define LIS3MDL_OUT_Z_L					0x2C
#define LIS3MDL_OUT_Z_H					0x2D

#define LIS3MDL_TEMP_OUT_L				0x2E
#define LIS3MDL_TEMP_OUT_H				0x2F

#define LIS3MDL_INT_CFG					0x30
#define LIS3MDL_INT_SRC					0x31
#define LIS3MDL_THS_L					0x32
#define LIS3MDL_THS_H					0x33


#define LIS3MDL_WHO_AM_I_RESPONSE		0b00111101

#define LIS3MDL_CTRL_REG1_1000HZ		0b00000010
#define LIS3MDL_CTRL_REG1_560HZ			0b00100010
#define LIS3MDL_CTRL_REG1_300HZ			0b01000010
#define LIS3MDL_CTRL_REG1_150HZ			0b01100010
#define LIS3MDL_CTRL_REG1_80HZ			0b01011100
#define LIS3MDL_CTRL_REG1_80HZ_LP			0b00011100

#define LIS3MDL_CTRL_REG2_4G			0b00000000
#define LIS3MDL_CTRL_REG2_8G			0b00100000
#define LIS3MDL_CTRL_REG2_12G			0b01000000
#define LIS3MDL_CTRL_REG2_16G			0b01100000

#define LIS3MDL_CTRL_REG3_CONT_MODE			0x0
#define LIS3MDL_CTRL_REG5_FAST			0b10000000
#define LIS3MDL_CTRL_REG5_S			0b00000000




#define LIS3MDL_PERFORMANCE_LOW_POWER  0b00
#define LIS3MDL_PERFORMANCE_MEDIUM     0b01
#define LIS3MDL_PERFORMANCE_HIGH       0b10
#define LIS3MDL_PERFORMANCE_ULTRA_HIGH 0b11

#define LIS3MDL_DATA_RATE_0_625_HZ 0b000
#define LIS3MDL_DATA_RATE_1_25_HZ  0b001
#define LIS3MDL_DATA_RATE_2_5_HZ   0b010
#define LIS3MDL_DATA_RATE_5_HZ     0b011
#define LIS3MDL_DATA_RATE_10_HZ    0b100
#define LIS3MDL_DATA_RATE_20_HZ    0b101
#define LIS3MDL_DATA_RATE_40_HZ    0b110
#define LIS3MDL_DATA_RATE_80_HZ    0b111

#define LIS3MDL_MODE_CONTINUOUS    0b00
#define LIS3MDL_MODE_SINGLE        0b01
#define LIS3MDL_MODE_POWER_DOWN    0b11

#define LIS3MDL_SCALE_4_GAUSS      0b00
#define LIS3MDL_SCALE_8_GAUSS      0b01
#define LIS3MDL_SCALE_12_GAUSS     0b10
#define LIS3MDL_SCALE_16_GAUSS     0b11

#define LIS3MDL_AXIS_X             0
#define LIS3MDL_AXIS_Y             1
#define LIS3MDL_AXIS_Z             2

#define LIS3MDL_STATUS_ZYXOR       0b10000000
#define LIS3MDL_STATUS_ZOR         0b01000000
#define LIS3MDL_STATUS_YOR         0b00100000
#define LIS3MDL_STATUS_XOR         0b00010000
#define LIS3MDL_STATUS_ZYXDA       0b00001000
#define LIS3MDL_STATUS_ZDA         0b00000100
#define LIS3MDL_STATUS_YDA         0b00000010
#define LIS3MDL_STATUS_XDA         0b00000001

#define LIS3MDL_DEVICE_ID          0b00111101

//MAG_BIAS: -0.0669 -0.2621 -0.1124  MAG_SCALE: 1.0150 0.9927 0.9927

//MAG_BIAS: 0.0307 -0.2255 -0.1124 MAG_SCALE: 1.0330 1.0246 0.9470

//MAG_BIAS: 0.0398 -0.1880 -0.1122 MAG_SCALE: 0.9596 1.0206 1.0224

/*#define MAG_BIAS_X 	0.031
#define MAG_BIAS_Y	-0.2252
#define MAG_BIAS_Z	-0.1123

#define MAG_SCALE_X	1.0025
#define MAG_SCALE_Y	1.0126
#define MAG_SCALE_Z	0.988*/
#define MAG_BIAS_X 	0.009
#define MAG_BIAS_Y	-0.09295
#define MAG_BIAS_Z	-0.2262495

#define MAG_SCALE_X	0.99269754
#define MAG_SCALE_Y	1.0112589
#define MAG_SCALE_Z	0.996236

typedef struct {
	uint8_t M_range;
	uint16_t adrs;
	uint16_t timeout;
	I2C_HandleTypeDef* i2c_channel;
} LIS3MDL_configuration;

typedef struct {
	int16_t M_x;
	int16_t M_y;
	int16_t M_z;
	float Mg_x;
	float Mg_y;
	float Mg_z;
} LIS3MDL_MAG_measurements;

typedef struct {
	float m[3];
} LIS3MDL_MAG;

typedef enum {
	LIS3MDL_OK = 0,
	LIS3MDL_FAIL = 1,
	LIS3MDL_FAIL_COMPARE = 2,
	LIS3MDL_FAIL_PARAMETERS = 3,
} LIS3MDL_Result;

LIS3MDL_configuration LIS3MDL_parameters;
LIS3MDL_MAG_measurements LIS3MDL_data;
LIS3MDL_MAG LIS3MDL;

void LIS3MDL_Init(uint8_t SA1, uint16_t timeout, I2C_HandleTypeDef* i2c_channel);
LIS3MDL_Result LIS3MDL_Configure(uint8_t efforts, uint8_t M_rate, uint8_t M_scale);
LIS3MDL_Result LIS3MDL_Detect(uint8_t efforts);
LIS3MDL_Result LIS3MDL_ReadStatus(void);

LIS3MDL_Result LSM6DS3_MAG_GetMeasurements(void);
LIS3MDL_Result LIS3MDL_MAG_GetMeasurements_gauss(void);




#endif /* INC_LIS3MDL_H_ */
