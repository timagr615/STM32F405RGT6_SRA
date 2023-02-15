/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LSM_INT2_Pin GPIO_PIN_0
#define LSM_INT2_GPIO_Port GPIOC
#define LSM_INT2_EXTI_IRQn EXTI0_IRQn
#define LSM_INT1_Pin GPIO_PIN_1
#define LSM_INT1_GPIO_Port GPIOC
#define LIS_DRDY_Pin GPIO_PIN_2
#define LIS_DRDY_GPIO_Port GPIOC
#define LIS_DRDY_EXTI_IRQn EXTI2_IRQn
#define BAT_LWL_Pin GPIO_PIN_4
#define BAT_LWL_GPIO_Port GPIOA
#define VLCD_Pin GPIO_PIN_6
#define VLCD_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOC
#define BTN0_Pin GPIO_PIN_5
#define BTN0_GPIO_Port GPIOC
#define BTN2_Pin GPIO_PIN_0
#define BTN2_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_1
#define BTN1_GPIO_Port GPIOB
#define Boot1_Pin GPIO_PIN_2
#define Boot1_GPIO_Port GPIOB
#define LORA_CS_Pin GPIO_PIN_12
#define LORA_CS_GPIO_Port GPIOB
#define LORA_RST_Pin GPIO_PIN_6
#define LORA_RST_GPIO_Port GPIOC
#define LORA_IRQ_Pin GPIO_PIN_7
#define LORA_IRQ_GPIO_Port GPIOC
#define USB_CONN_Pin GPIO_PIN_10
#define USB_CONN_GPIO_Port GPIOA
#define SDIO_Detect_Pin GPIO_PIN_15
#define SDIO_Detect_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define DEBUG 1

#define DEBUG_LWL 1


typedef enum {
	IO_Read,
	IO_Write
} IO_Operation;


typedef struct {
	IO_Operation op;
	uint32_t *buf;
	uint32_t blk_addr;
	uint16_t blk_len;
	void *context;
} IO_Msg;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
