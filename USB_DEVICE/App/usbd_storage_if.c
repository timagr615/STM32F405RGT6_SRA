/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_storage_if.c
  * @version        : v1.0_Cube
  * @brief          : Memory management layer.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_storage_if.h"

/* USER CODE BEGIN INCLUDE */
#include "bsp_driver_sd.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "main.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern SD_HandleTypeDef hsd;
extern QueueHandle_t sdCmdQueue;
extern IO_Msg;
extern IO_Operation;
//extern HAL_SD_CardInfoTypedef SDCardInfo;
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @defgroup USBD_STORAGE
  * @brief Usb mass storage device module
  * @{
  */

/** @defgroup USBD_STORAGE_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Defines
  * @brief Private defines.
  * @{
  */

#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_NBR                  0x10000
#define STORAGE_BLK_SIZ                  0x200

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN INQUIRY_DATA_FS */
/** USB Mass storage Standard Inquiry Data. */
const int8_t STORAGE_Inquirydata_FS[] = {/* 36 */

  /* LUN 0 */
  0x00,
  0x80,
  0x02,
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,
  0x00,
  'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
  'P', 'r', 'o', 'd', 'u', 'c', 't', ' ', /* Product      : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0' ,'1'                      /* Version      : 4 Bytes */
};
/* USER CODE END INQUIRY_DATA_FS */

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t STORAGE_Init_FS(uint8_t lun);
static int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
static int8_t STORAGE_IsReady_FS(uint8_t lun);
static int8_t STORAGE_IsWriteProtected_FS(uint8_t lun);
static int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len, void *context);
static int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len, void *context);
static int8_t STORAGE_GetMaxLun_FS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_StorageTypeDef USBD_Storage_Interface_fops_FS =
{
  STORAGE_Init_FS,
  STORAGE_GetCapacity_FS,
  STORAGE_IsReady_FS,
  STORAGE_IsWriteProtected_FS,
  STORAGE_Read_FS,
  STORAGE_Write_FS,
  STORAGE_GetMaxLun_FS,
  (int8_t *)STORAGE_Inquirydata_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the storage unit (medium) over USB FS IP
  * @param  lun: Logical unit number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Init_FS(uint8_t lun)
{
  /* USER CODE BEGIN 2 */
	uint8_t res = BSP_SD_Init();
	//UNUSED(lun);
	printf("BSD INIT: %d \r\n");
	return (USBD_OK);
  /* USER CODE END 2 */
}

/**
  * @brief  Returns the medium capacity.
  * @param  lun: Logical unit number.
  * @param  block_num: Number of total block number.
  * @param  block_size: Block size.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  /* USER CODE BEGIN 3 */
  /*UNUSED(lun);

  *block_num  = STORAGE_BLK_NBR;
  *block_size = STORAGE_BLK_SIZ;
  return (USBD_OK);*/
	/*HAL_SD_CardInfoTypeDef info;
	int8_t ret = -1;



	BSP_SD_GetCardInfo(&info);
	*block_num = info.LogBlockNbr - 1;
	*block_size = info.LogBlockSize;
	ret = 0;
	return ret;*/
	/*HAL_SD_CardInfoTypeDef info;
	  int8_t ret = -1;

	  uint8_t res = HAL_SD_GetCardInfo(&hsd, &info);
	  printf("res:  %d   Card_Type %d \r\n", res, info.CardType);

	  *block_num =  info.LogBlockNbr  - 1;
	  *block_size = info.LogBlockSize;
	  ret = 0;
	  return ret;*/


	HAL_SD_CardInfoTypeDef info;
	  int8_t ret = 0;

	  BSP_SD_GetCardInfo(&info);

	  *block_num  = info.LogBlockNbr;
	  *block_size = info.LogBlockSize;

	  return ret;
  /* USER CODE END 3 */
}

/**
  * @brief   Checks whether the medium is ready.
  * @param  lun:  Logical unit number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsReady_FS(uint8_t lun)
{
  /* USER CODE BEGIN 4 */
  //UNUSED(lun);

  //return (USBD_OK);
	static int8_t prev_status = 0;
	  int8_t ret = -1;

	  if(prev_status < 0)
	  {
	    BSP_SD_Init();
	    prev_status = 0;
	  }
	  if(BSP_SD_GetCardState() == MSD_OK)
	  {
	    ret = 0;
	  }

	  return ret;
  /* USER CODE END 4 */
}

/**
  * @brief  Checks whether the medium is write protected.
  * @param  lun: Logical unit number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsWriteProtected_FS(uint8_t lun)
{
  /* USER CODE BEGIN 5 */
  UNUSED(lun);

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Reads data from the medium.
  * @param  lun: Logical unit number.
  * @param  buf: data buffer.
  * @param  blk_addr: Logical block address.
  * @param  blk_len: Blocks number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len, void * context)
{
  /* USER CODE BEGIN 6 */
	IO_Msg msg;
	msg.op = IO_Read;
	msg.blk_addr = blk_addr;
	msg.blk_len = blk_len;
	msg.buf =(uint32_t *) buf;
	msg.context = context;

	if(xQueueSendFromISR(sdCmdQueue, &msg, pdFALSE) != pdPASS)
			return USBD_FAIL;

	return (USBD_OK);


  /*UNUSED(lun);
  UNUSED(buf);
  UNUSED(blk_addr);
  UNUSED(blk_len);

  return (USBD_OK);*/
	/*int8_t ret = -1;
	BSP_SD_ReadBlocks((uint32_t *) buf, blk_addr, blk_len, 10);


	while (BSP_SD_GetCardState() != SD_TRANSFER_OK){}
	ret = 0;
	return ret;*/



	/*int8_t ret = -1;

	  HAL_SD_ReadBlocks(&hsd, buf, blk_addr, blk_len, HAL_MAX_DELAY);


	  while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER){}
	  ret = 0;
	  return ret;*/

	/*int8_t ret = -1;
	  uint32_t timeout = 100000;
	  BSP_SD_ReadBlocks((uint32_t *)buf, blk_addr, blk_len, timeout);
	  while(BSP_SD_GetCardState() != MSD_OK)
	  {
	    if (timeout-- == 0)
	    {
	      return ret;
	    }
	  }
	  ret = 0;

	  return ret;*/

  /* USER CODE END 6 */
}

/**
  * @brief  Writes data into the medium.
  * @param  lun: Logical unit number.
  * @param  buf: data buffer.
  * @param  blk_addr: Logical block address.
  * @param  blk_len: Blocks number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len, void *context)
{
  /* USER CODE BEGIN 7 */
	IO_Msg msg;
	msg.op = IO_Write;
	msg.blk_addr = blk_addr;
	msg.blk_len = blk_len;
	msg.buf =(uint32_t *) buf;
	msg.context = context;

	if(xQueueSendFromISR(sdCmdQueue, &msg, pdFALSE) != pdPASS)
		return USBD_FAIL;
	return (USBD_OK);

  /*UNUSED(lun);
  UNUSED(buf);
  UNUSED(blk_addr);
  UNUSED(blk_len);

  return (USBD_OK);*/

	/* int8_t ret = -1;

	 BSP_SD_WriteBlocks((uint32_t *) buf, blk_addr, blk_len, 10);


	 while (BSP_SD_GetCardState() != SD_TRANSFER_OK){}
	 ret = 0;

	 return ret;*/



	/*int8_t ret = -1;

	   HAL_SD_WriteBlocks(&hsd, buf, blk_addr, blk_len, HAL_MAX_DELAY);



	  while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER){}
	  ret = 0;
	  return ret;*/
	/*int8_t ret = -1;
	  uint32_t timeout = 100000;
	  BSP_SD_WriteBlocks((uint32_t *)buf, blk_addr, blk_len, timeout);
	  while(BSP_SD_GetCardState() != MSD_OK)
	  {
	    if (timeout-- == 0)
	    {
	      return ret;
	    }
	  }
	  ret = 0;

	  return ret;*/
  /* USER CODE END 7 */
}

/**
  * @brief  Returns the Max Supported LUNs.
  * @param  None
  * @retval Lun(s) number.
  */
int8_t STORAGE_GetMaxLun_FS(void)
{
  /* USER CODE BEGIN 8 */
  return (STORAGE_LUN_NBR - 1);
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

