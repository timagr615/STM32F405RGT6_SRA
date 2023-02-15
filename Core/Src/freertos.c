/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "usb_device.h"
#include "bsp_driver_sd.h"
#include "fatfs.h"
#include <stdio.h>
#include "GNSS.h"
#include "lsm6ds3.h"
#include "lis3mdl.h"
#include "imu.h"
#include "coordinates.h"
#include "fusion.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

extern void cardReadCompletedCB(uint8_t res, void * context);
extern void cardWriteCompletedCB(uint8_t res, void * context);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KF_STATE_DIM 4
#define KF_OBS_DIM 4
#define KF_U_DIM 2
#define KF_INIT_POSITION_VAR 150.0
#define KF_INIT_VELOCITY_VAR 15.0
#define KF_SIGM_ACC 0.2


float mx = 0.0;
float my = 0.0;
float mz = 0.0;
float gx, gy, gz;
float ax = 0.0;
float ay = 0.0;
float az = 0.0;
float axi = 0.0;
float ayi = 0.0;
float azi = 0.0;
float r = 0.0;
float p = 0.0;
float y = 0.0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static TaskHandle_t xTaskGpsProcess = NULL;
static TaskHandle_t xTaskLSMProcess = NULL;
static TaskHandle_t xTaskLISProcess = NULL;
static TaskHandle_t xTaskIMUProcess = NULL;
static TaskHandle_t xDebugHandle = NULL;

QueueHandle_t sdCmdQueue = NULL;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskPolling */
osThreadId_t taskPollingHandle;
const osThreadAttr_t taskPolling_attributes = {
  .name = "taskPolling",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskDebug */
osThreadId_t taskDebugHandle;
const osThreadAttr_t taskDebug_attributes = {
  .name = "taskDebug",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskGpsProcess */
osThreadId_t taskGpsProcessHandle;
const osThreadAttr_t taskGpsProcess_attributes = {
  .name = "taskGpsProcess",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskLSM */
osThreadId_t taskLSMHandle;
const osThreadAttr_t taskLSM_attributes = {
  .name = "taskLSM",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskLIS */
osThreadId_t taskLISHandle;
const osThreadAttr_t taskLIS_attributes = {
  .name = "taskLIS",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for taskIMU */
osThreadId_t taskIMUHandle;
const osThreadAttr_t taskIMU_attributes = {
  .name = "taskIMU",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskGPSFilter */
osThreadId_t taskGPSFilterHandle;
const osThreadAttr_t taskGPSFilter_attributes = {
  .name = "taskGPSFilter",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskFusion */
osThreadId_t taskFusionHandle;
const osThreadAttr_t taskFusion_attributes = {
  .name = "taskFusion",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskUSB */
osThreadId_t taskUSBHandle;
const osThreadAttr_t taskUSB_attributes = {
  .name = "taskUSB",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

long int lsmRate = 0;
int lisRate = 0;
void printBits(size_t const size, void const * const ptr)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;

    for (i = size-1; i >= 0; i--) {
        for (j = 7; j >= 0; j--) {
            byte = (b[i] >> j) & 1;
            printf("%u", byte);
        }
    }
    printf(" ");
}

void vGPSPollingInterruptHandler( void );
void vLSMPollingInterruptHandler(void);
void vLISPollingInterruptHandler(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTaskGpsPolling(void *argument);
void StartTaskDebug(void *argument);
void StartTaskGpsProcess(void *argument);
void StartTaskLSM(void *argument);
void StartTaskLIS(void *argument);
void StartTaskIMU(void *argument);
void StartTaskGPSFilter(void *argument);
void StartTaskFusion(void *argument);
void StartTaskUSB(void *argument);

void StartTaskTEST(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */

/*void mag_calibration(){
	printf("MAGNETOMETER CALIBRATION START in 3 seconds\r\n");
	osDelay(3000);
	uint16_t ii = 0;
	uint16_t sample_count = 10000;
	float mag_bias[3] = {0.0};
	float mag_scale[3] = {0.0};
	float max_mag[3] = {-8.0, -8.0, -8.0};
	float min_mag[3] = {8.0, 8.0, 8.0};
	float mag_temp[3] = {0.0};
	LIS3MDL_MAG_GetMeasurements_gauss();
	while(ii <= sample_count){
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
		LIS3MDL_MAG_GetMeasurements_gauss();

		for (uint8_t j = 0; j < 3; j++){
			mag_temp[j] = LIS3MDL.m[j];
			if(mag_temp[j] > max_mag[j]){
				max_mag[j] = mag_temp[j];
			}
			if(mag_temp[j] < min_mag[j]){
				min_mag[j] = mag_temp[j];
			}
		}
		ii ++;
	}
	for (uint8_t i = 0; i < 3; i++){
		mag_bias[i] = (max_mag[i] + min_mag[i])/2.0;
		mag_scale[i]  = (max_mag[i] - min_mag[i])/2.0;
	}
	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	for (uint8_t i = 0; i < 3; i++){
		mag_scale[i]  = avg_rad/mag_scale[i];
	}
	printf("MAG_BIAS: %.4f %.4f %.4f MAG_SCALE: %.4f %.4f %.4f ", mag_bias[0], mag_bias[1], mag_bias[2], mag_scale[0], mag_scale[1], mag_scale[2]);
	xTaskNotifyGive(xTaskGpsProcess);
	xTaskNotifyGive(xTaskLSMProcess);
}*/
/*void acc_gyro_calibration(){
	printf("ACC GYRO CALIBRATING START \r\n");
		osDelay(3000);
		LSM6DS3_IMU_GetData();
		int data_count = 0;
		float ac[3] = {0.0};
		float gc[3] = {0.0};

		while (data_count < 10000){
			ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
			LSM6DS3_IMU_GetData();
			for (uint8_t i = 0; i < 3; i++){
				ac[i] += LSM6DS3.a[i];
				gc[i] += LSM6DS3.g[i];
			}
			data_count++;
		}
		for (uint8_t j = 0; j < 3; j++){
			ac[j] /= 10000.0;
			gc[j] /= 10000.0;
		}
		printf("BIASES ACC: %.4f %.4f %.4f GYRO: %.4f %.4f %.4f \r\n", ac[0], ac[1], ac[2], gc[0], gc[1], gc[2]);
		xTaskNotifyGive(xTaskGpsProcess);
		xTaskNotifyGive(xTaskLISProcess);
}*/

void vLSMPollingInterruptHandler(void){

	lsmRate += 1;
	//BaseType_t xHigherPriorityTaskWoken;

	//xHigherPriorityTaskWoken = pdFALSE;


	//vTaskNotifyGiveFromISR( xTaskLSMProcess, &xHigherPriorityTaskWoken );


	//portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}



void vLISPollingInterruptHandler(void){

	lisRate += 1;
	//printf("LIS INTERRUPT \r\n");
	BaseType_t xHigherPriorityTaskWoken1;

	xHigherPriorityTaskWoken1 = pdFALSE;


	vTaskNotifyGiveFromISR( taskLISHandle, &xHigherPriorityTaskWoken1 );


	portYIELD_FROM_ISR( xHigherPriorityTaskWoken1 );

}
void vGPSPollingInterruptHandler( void )
{

    BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;


    vTaskNotifyGiveFromISR( taskGpsProcessHandle, &xHigherPriorityTaskWoken );


    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of taskPolling */
  taskPollingHandle = osThreadNew(StartTaskGpsPolling, NULL, &taskPolling_attributes);

  /* creation of taskDebug */
  taskDebugHandle = osThreadNew(StartTaskDebug, NULL, &taskDebug_attributes);

  /* creation of taskGpsProcess */
  taskGpsProcessHandle = osThreadNew(StartTaskGpsProcess, NULL, &taskGpsProcess_attributes);

  /* creation of taskLSM */
  taskLSMHandle = osThreadNew(StartTaskLSM, NULL, &taskLSM_attributes);

  /* creation of taskLIS */
  taskLISHandle = osThreadNew(StartTaskLIS, NULL, &taskLIS_attributes);

  /* creation of taskIMU */
  taskIMUHandle = osThreadNew(StartTaskIMU, NULL, &taskIMU_attributes);

  /* creation of taskGPSFilter */
  taskGPSFilterHandle = osThreadNew(StartTaskGPSFilter, NULL, &taskGPSFilter_attributes);

  /* creation of taskFusion */
  taskFusionHandle = osThreadNew(StartTaskFusion, NULL, &taskFusion_attributes);

  /* creation of taskUSB */
  taskUSBHandle = osThreadNew(StartTaskUSB, NULL, &taskUSB_attributes);



  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}


/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
	MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  //uint8_t res = 0;
  /*res = f_mount(&SDFatFS, SDPath, 1);
    	  	if(res != FR_OK)
    	  	{
    	  		printf("ERROR in mount filesystem \r\n");
    	  	}
    	  	else
    	  	{
    	  		printf("Success mount filesystem \r\n");
    	  	}*/
	//BSP_SD_Init();
	//MX_FATFS_Init();
	//MX_USB_DEVICE_Init();
	/*HAL_SD_CardInfoTypeDef Card_Info;
	int8_t const card_type[3][15] = {"CARD_SDSC\0","CARD_SDHC_SDXC\0","CARD_SECURED\0"};
	int8_t const card_ver [2][10] = {"CARD_V1_X\0","CARD_V1_X\0"};
	volatile FRESULT fres;

	printf("\n\rStart testing SDCARD\n\r");
	BSP_SD_GetCardInfo(&Card_Info);
	printf("Card Type               -> %s\n", card_type[Card_Info.CardType]);
	printf("Card Version            -> %s\n", card_ver[Card_Info.CardVersion]);
	printf("Block Size              -> 0x%x\n", (int)Card_Info.BlockSize);
	printf("Card Capacity in blocks -> 0x%x(%uGB)\n\r", (int)Card_Info.BlockNbr,(int)((((float)Card_Info.BlockNbr/1000)*(float)Card_Info.BlockSize/1000000)+0.5));
	osDelay(5000);

	fres = f_mount(&SDFatFS, (const TCHAR*)SDPath, 0);
	printf("Mount FAT FS request sent\n");

	printf("Formatting storage device\n");
	void* work = malloc(512);
	fres = f_mkfs((const TCHAR*)SDPath, FM_FAT32, 0, work, 512);
	if (fres != FR_OK) {
	    printf("Formatting SD failed\n");
	}
	else {
	    printf("SD formatted\n");
	}
	free(work);

	printf("Opening file\n");

	fres = f_open(&SDFile, (const TCHAR*)"UMC2023-02-13-.log", FA_CREATE_NEW | FA_WRITE);
	if (fres == FR_OK) {
	    printf ("File opened\n");
	    f_close(&SDFile);
	}
	else {
	    printf("Error opening file. ExitCode = %i\n", fres);
	}
	osDelay(5000);*/
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskGpsPolling */
/**
* @brief Function implementing the taskPolling thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskGpsPolling */
void StartTaskGpsPolling(void *argument)
{
  /* USER CODE BEGIN StartTaskGpsPolling */
  /* Infinite loop */
	GNSS_Init(&GNSS_Handle, &huart2);
	GNSS_LoadConfig(&GNSS_Handle);
	osDelay(100);
  for(;;)
  {
	  GNSS_GetPVTData(&GNSS_Handle);
	  osDelay(1000);
  }
  /* USER CODE END StartTaskGpsPolling */
}

/* USER CODE BEGIN Header_StartTaskDebug */
/**
* @brief Function implementing the taskDebug thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskDebug */
void StartTaskDebug(void *argument)
{
  /* USER CODE BEGIN StartTaskDebug */
	configASSERT( xDebugHandle == NULL);
	xDebugHandle = xTaskGetCurrentTaskHandle();
  /* Infinite loop */
  for(;;)
  {
	  ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	  //printf("ACC: %0.4f %0.4f %0.4f \r\n", IMU.a[0], LSM6DS3.a[1], LSM6DS3.a[2]);
	  //printf("MAG: %0.4f %0.4f %0.4f \r\n", LIS3MDL.m[0], LIS3MDL.m[1], LIS3MDL.m[2]);
	  printf("IMU ACC: %0.4f %0.4f %0.4f \r\n", IMU.rpy[0], IMU.rpy[1], IMU.rpy[2]);
	  //printf("LisRATE: %d \r\n", lisRate);
	  //printf("LsmRATE: %d \r\n", lsmRate);
	  if (DEBUG && DEBUG_LWL <= 0){
		  //printf("LSM RATE INTERRUPT %d \r\n", lsmRate);
		  printf("Day: %d-%d-%d \r\n", GNSS_Handle.day, GNSS_Handle.month,GNSS_Handle.year);
		  printf("Time: %d:%d:%d \r\n", GNSS_Handle.hour, GNSS_Handle.min,GNSS_Handle.sec);
		  printf("Status of fix: %d \r\n", GNSS_Handle.fixType);
		  printf("Sat number: %d \r\n", GNSS_Handle.numSV);
		  printf("Latitude: %f \r\n", GNSS_Handle.fLat);
		  printf("Longitude: %f \r\n",GNSS_Handle.fLon);
		  printf("Course: %.2f \r\n", GNSS_Handle.course);
		  printf("Ground Speed (2-D): %.2f \r\n", GNSS_Handle.gSpeed);
		  printf("velN velE velD: %.2f %.2f %.2f \r\n", GNSS_Handle.velN, GNSS_Handle.velE, GNSS_Handle.velD);

		  printf("Horizontal, speed, course accuracy: %.2f %.2f %.2f \r\n", GNSS_Handle.hAcc, GNSS_Handle.sAcc, GNSS_Handle.headAcc);
		  /*printf("Unique ID: %04X %04X %04X %04X %04X \n\r",
					  GNSS_Handle.uniqueID[0], GNSS_Handle.uniqueID[1],
					  GNSS_Handle.uniqueID[2], GNSS_Handle.uniqueID[3],
					  GNSS_Handle.uniqueID[4], GNSS_Handle.uniqueID[5]);*/
	  }
	  //osDelay(1000);
  }
  /* USER CODE END StartTaskDebug */
}

/* USER CODE BEGIN Header_StartTaskGpsProcess */
/**
* @brief Function implementing the taskGpsProcess thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskGpsProcess */
void StartTaskGpsProcess(void *argument)
{
  /* USER CODE BEGIN StartTaskGpsProcess */
	configASSERT( xTaskGpsProcess == NULL);
	xTaskGpsProcess = xTaskGetCurrentTaskHandle();
	//ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
  /* Infinite loop */
  for(;;)
  {
	  ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	  GNSS_ParseBuffer(&GNSS_Handle);
	  if (GPS_Filter.firstFix == 0){
		  if(GNSS_Handle.fixType > 1){
			  GPS_Filter.firstFix = 1;
			  GPS_Filter.oldFixmode = GNSS_Handle.fixType;
			  GPS_Filter.oldHacc = GNSS_Handle.hAcc;
			  GEO_Point_set(&GNSS_Handle, &pointFirst, 1, 1);
			  GEO_Point_set(&GNSS_Handle, &pointOld, 1, 1);
			  GEO_Point_set(&GNSS_Handle, &pointCurrent, 1, 1);
		  }
	  }


	  xTaskNotifyGive(taskGPSFilterHandle);
	  xTaskNotifyGive( taskDebugHandle );
	  //osDelay(1);
  }
  /* USER CODE END StartTaskGpsProcess */
}

/* USER CODE BEGIN Header_StartTaskLSM */
/**
* @brief Function implementing the taskLSM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLSM */
void StartTaskLSM(void *argument)
{
  /* USER CODE BEGIN StartTaskLSM */
  /* Infinite loop */
	configASSERT( xTaskLSMProcess == NULL);
	xTaskLSMProcess = xTaskGetCurrentTaskHandle();
	//ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	osDelay(100);
	LSM6DS3_Init(LSM6DS3_SA0_HIGH_LEVEL, LSM6DS3_DEFAULT_TIMEOUT*10, &hi2c1);
	LSM6DS3_Configure(10, XL_1666Hz, XL_RANGE_4G, XL_FILTER_400Hz, GS_1666Hz, GS_RANGE_250dps);
	osDelay(10);
	LSM6DS3_IMU_GetData();


  for(;;)
  {
	  ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	  //printf("LSM task after notification \r\n");
	  if(LSM6DS3_IMU_GetData() == LSM6DS3_OK){
		  /*ax = LSM6DS3.a[0];
		  ay = LSM6DS3.a[1];
		  az = LSM6DS3.a[2];
		  gx = LSM6DS3.g[0];
		  gy = LSM6DS3.g[1];
		  gz = LSM6DS3.g[2];*/

	  }
	  xTaskNotifyGive( taskIMUHandle );
	  //printf("LSM LOOP \r\n");



  }
  /* USER CODE END StartTaskLSM */
}

/* USER CODE BEGIN Header_StartTaskLIS */
/**
* @brief Function implementing the taskLIS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLIS */
void StartTaskLIS(void *argument)
{
  /* USER CODE BEGIN StartTaskLIS */
  /* Infinite loop */
	configASSERT( xTaskLISProcess == NULL);
	xTaskLISProcess = xTaskGetCurrentTaskHandle();
	//ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	//float ax, ay, az, gx, gy, gz;
	osDelay(100);
	LIS3MDL_Init(LIS3MDL_SA1_LOW_LEVEL, 100, &hi2c1);
	LIS3MDL_Configure(10, LIS3MDL_CTRL_REG1_80HZ_LP, LIS3MDL_CTRL_REG2_8G);
	osDelay(10);
	LIS3MDL_MAG_GetMeasurements_gauss();



  for(;;)
  {
	  ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	  //printf("LIS LOOP \r\n");

	  LIS3MDL_MAG_GetMeasurements_gauss();
	  //float r = sqrt(LIS3MDL.m[0]*LIS3MDL.m[0]+LIS3MDL.m[1]*LIS3MDL.m[1]+LIS3MDL.m[2]*LIS3MDL.m[2]);
	  //printf("%f %f %f \r\n", LIS3MDL.m[0], LIS3MDL.m[1], LIS3MDL.m[2]);
	  //printf("LIS task before notification give \r\n");
	  xTaskNotifyGive( taskLSMHandle );


    //osDelay(1000);
  }
  /* USER CODE END StartTaskLIS */
}

/* USER CODE BEGIN Header_StartTaskIMU */
/**
* @brief Function implementing the taskIMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskIMU */
void StartTaskIMU(void *argument)
{
  /* USER CODE BEGIN StartTaskIMU */
  /* Infinite loop */
	configASSERT( xTaskIMUProcess == NULL);
	xTaskIMUProcess = xTaskGetCurrentTaskHandle();
	osDelay(100);
	IMU_Init(&IMU);
  for(;;)
  {
	  ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	  IMU_Update(&IMU, &LIS3MDL, &LSM6DS3);
	  axi = IMU.long_acc;
	  ayi = IMU.earth_acc[0];
	  azi = IMU.earth_acc[1];

	  gx = IMU.gned[0];
	  gy = IMU.gned[1];
	  gz = IMU.gned[2];
	  mx = IMU.mned[0];
	  my = IMU.mned[1];
	  mz = IMU.mned[2];
	  r = IMU.rpy[0];
	  p = IMU.rpy[1];
	  y = IMU.rpy[2];
	  xTaskNotifyGive(taskFusionHandle);
  }
  /* USER CODE END StartTaskIMU */
}

/* USER CODE BEGIN Header_StartTaskGPSFilter */
/**
* @brief Function implementing the taskGPSFilter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskGPSFilter */
void StartTaskGPSFilter(void *argument)
{
  /* USER CODE BEGIN StartTaskGPSFilter */
	xTaskGetCurrentTaskHandle();
	GPS_Filter_Init(&GPS_Filter);

  /* Infinite loop */
  for(;;)
  {
	  ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	  /*
	   * На выходе обновляем две точки структуры GEO_Point:
	   * pointCurrent - самая новая точка,
	   * pointOld - последняя валидная точка
	   * Функция построена таким образом, что если с GPS приходит хорошая точка, то она сохраняется в pointCurrent с флагами
	   * success = 1 и isNew = 1, а также копируется в pointOld.
	   * Если же с GPS приходит плохая точка, то в pointCurrent попадает последняя хорошая точка(pointOld) с флагами
	   * success = 0 и isNew = 0
	   */
	  GPS_FilterUpdate(&GNSS_Handle, &GPS_Filter, &pointOld, &pointCurrent);
  }
  /* USER CODE END StartTaskGPSFilter */
}

/* USER CODE BEGIN Header_StartTaskFusion */
/**
* @brief Function implementing the taskFusion thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskFusion */
void StartTaskFusion(void *argument)
{
  /* USER CODE BEGIN StartTaskFusion */
	osDelay(1000);
	initFusion(&Fusion, &KF, KF_STATE_DIM, KF_OBS_DIM, KF_U_DIM, KF_INIT_POSITION_VAR, KF_INIT_VELOCITY_VAR, KF_SIGM_ACC);
	double dt = IMU.deltaT;
  /* Infinite loop */
  for(;;)
  {
	  ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	  if (GPS_Filter.firstFix == 1){
		  Fusion_compute(&Fusion, &IMU, &pointCurrent, dt);
	  }
  }
  /* USER CODE END StartTaskFusion */
}

/* USER CODE BEGIN Header_StartTaskUSB */
/**
* @brief Function implementing the taskUSB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskUSB */
void StartTaskUSB(void *argument)
{
  /* USER CODE BEGIN StartTaskUSB */
	//MX_FATFS_Init();
	//MX_USB_DEVICE_Init();
	sdCmdQueue = xQueueCreate(1, sizeof(IO_Msg));
	//BSP_SD_Init();
	//MX_FATFS_Init();
	//MX_USB_DEVICE_Init();
  /* Infinite loop */
  for(;;)
  {

	  IO_Msg msg;
	  if(xQueueReceive(sdCmdQueue, &msg, portMAX_DELAY)){
		  switch(msg.op){
			  case IO_Read:
			  {
				  uint32_t timeout = 10;
				  uint8_t res = BSP_SD_ReadBlocks(msg.buf, msg.blk_addr, msg.blk_len, timeout);
				  //printf("res after ReadBlocks %d \r\n", res);
				  res = 0;
				  while(BSP_SD_GetCardState() != MSD_OK)
				  	  {
				  	    if (timeout-- == 0)
				  	    {
				  	      res = 1;
				  	      break;
				  	    }
				  	  }
				  	  //res = 0;
				  //printf("res: %d \r\n", res);
				  cardReadCompletedCB(res, msg.context);
				  break;
			  }
			  case IO_Write:
			  {
				  uint32_t timeout = 10;
				  uint8_t res = BSP_SD_WriteBlocks(msg.buf, msg.blk_addr, msg.blk_len, timeout);
				  //printf("res after WriteBlocks %d \r\n", res);
				  res = 0;
				  while(BSP_SD_GetCardState() != MSD_OK)
				  				  	  {
				  				  	    if (timeout-- == 0)
				  				  	    {
				  				  	      res = 1;
				  				  	      break;
				  				  	    }
				  				  	  }
				  				  	  //res = 0;
				  				  	//printf("res: %d \r\n", res);
				  cardWriteCompletedCB(res, msg.context);
				  break;
			  }
			  default:
				  break;
		  }
	  }
	  //osDelay(1000);
  }
  /* USER CODE END StartTaskUSB */
}




/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

