/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "i2c.h"
#include "mpu.h"
#include "lcd.h"
#include "3DBox.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	float pitch, roll, yaw;
} mpu_data_t;

typedef struct {
	uint32_t cmd;
	uint32_t arg[3];
} bt_data_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osMailQId mail_sensorsHandle;
osMailQId mail_cmdHandle;
/* USER CODE END Variables */
osThreadId thread_sensorsHandle;
osThreadId thread_lcdHandle;
osThreadId thread_btHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void thread_sensors_func(void const * argument);
void thread_lcd_func(void const * argument);
void thread_bt_func(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  osMailQDef (mail_sensors, 10, mpu_data_t);  // Declare mail queue
  mail_sensorsHandle = osMailCreate(osMailQ(mail_sensors), NULL);

  osMailQDef (mail_cmd, 10, bt_data_t);  // Declare mail queue
  mail_cmdHandle = osMailCreate(osMailQ(mail_cmd), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of thread_sensors */
  osThreadDef(thread_sensors, thread_sensors_func, osPriorityHigh, 0, 512);
  thread_sensorsHandle = osThreadCreate(osThread(thread_sensors), NULL);

  /* definition and creation of thread_lcd */
  osThreadDef(thread_lcd, thread_lcd_func, osPriorityLow, 0, 512);
  thread_lcdHandle = osThreadCreate(osThread(thread_lcd), NULL);

  /* definition and creation of thread_bt */
  osThreadDef(thread_bt, thread_bt_func, osPriorityNormal, 0, 128);
  thread_btHandle = osThreadCreate(osThread(thread_bt), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_thread_sensors_func */
/**
  * @brief  Function implementing the thread_sensors thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_thread_sensors_func */
void thread_sensors_func(void const * argument)
{
  /* USER CODE BEGIN thread_sensors_func */
	#define RAD2DEG (180.0f / 3.14159f)
	#define DEG2RAD (3.14159f / 180.0f)

	uint32_t counter = 0;

	/* Variables for all measurements */
	int32_t quat[4] = { };
	uint16_t accScale = 0;
	float gyroScale = 0.f;
	uint8_t morePackets = 0;
	int8_t orientation[] = { 1, 0, 0,
							0, 1, 0,
							0, 0, 1 };
	float qw, qx, qy, qz;
	float dmpPRY[3];	// Angle calculated using DMP

	/* Timestamp & status variables */
	int16_t intStatus;

	/* Start MPU9250 and change settings */
	mpu_i2c_init(&hi2c1);
	if (mpu_init(NULL)) {
		osThreadTerminate(NULL);
	}
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	mpu_set_sample_rate(400);
	mpu_get_accel_sens(&accScale);
	mpu_get_gyro_sens(&gyroScale);
	mpu_set_lpf(100);

	/* Enable DMP */
	dmp_load_motion_driver_firmware();
	dmp_set_orientation(inv_orientation_matrix_to_scalar(orientation));
	dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_TAP);
	dmp_set_fifo_rate(200);
	mpu_set_dmp_state(1);

  /* Infinite loop */
  for(;;)
  {
	  /* Get current mpu chip status */
		mpu_get_int_status(&intStatus);
		if ((intStatus & MPU_INT_STATUS_DATA_READY) || morePackets > 0) {
			int16_t sensors = 0;
			/* Get quaternion */
			dmp_read_fifo(NULL, NULL, (long *) quat, NULL, &sensors, &morePackets);
			if (sensors & INV_WXYZ_QUAT) {
				qw = inv_q30_to_float(quat[0]);
				qx = inv_q30_to_float(quat[1]);
				qy = inv_q30_to_float(quat[2]);
				qz = inv_q30_to_float(quat[3]);
				if (qw < -1.0f) {
					qw = -1.0f;
				} else if (qw > 1.0f) {
					qw = 1.0f;
				}
				if (qx < -1.0f) {
					qx = -1.0f;
				} else if (qx > 1.0f) {
					qx = 1.0f;
				}
				if (qy < -1.0f) {
					qy = -1.0f;
				} else if (qy > 1.0f) {
					qy = 1.0f;
				}
				if (qz < -1.0f) {
					qz = -1.0f;
				} else if (qz > 1.0f) {
					qz = 1.0f;
				}
				dmpPRY[0] = asinf(-2.0f * (qx*qz - qw*qy));
				dmpPRY[1] = atan2f(qw*qx + qy*qz, 0.5f - qx*qx - qy*qy);
				dmpPRY[2] = atan2f(qx*qy + qw*qz, 0.5f - qy*qy - qz*qz) + 3.14159 / 2;
				counter++;
				/* Send every 8th sample */
				if (counter % 8 == 0) {
					/* Send data to queue */
					mpu_data_t * data = osMailAlloc(mail_sensorsHandle, 5);
					if (data != NULL) {
						data->pitch = dmpPRY[0];
						data->roll = dmpPRY[1];
						data->yaw = dmpPRY[2];
						osMailPut(mail_sensorsHandle, data);
					}
				}
			}
		}
		 osDelay(10);
  }
  /* USER CODE END thread_sensors_func */
}

/* USER CODE BEGIN Header_thread_lcd_func */
/**
* @brief Function implementing the thread_lcd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_thread_lcd_func */
void thread_lcd_func(void const * argument)
{
  /* USER CODE BEGIN thread_lcd_func */
	osEvent event;
	Object3d__HandleTypeDef box;
	LCD_Init();
	Object3d_InitBox(&box, 120, 80, 10);
  /* Infinite loop */
  for(;;)
  {
	  event = osMailGet(mail_sensorsHandle, 0);
	  if (event.status == osEventMail) {
		  mpu_data_t * data = event.value.p;
		  Object3d_CleanObject(&box);
		  Object3d_SetRotation(&box, data->pitch, data->roll, data->yaw);
		  Object3d_DrawObject(&box);
		  osMailFree(mail_sensorsHandle, data);
	  }
	  osDelay(1);
  }
  /* USER CODE END thread_lcd_func */
}

/* USER CODE BEGIN Header_thread_bt_func */
/**
* @brief Function implementing the thread_bt thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_thread_bt_func */
void thread_bt_func(void const * argument)
{
  /* USER CODE BEGIN thread_bt_func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END thread_bt_func */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
