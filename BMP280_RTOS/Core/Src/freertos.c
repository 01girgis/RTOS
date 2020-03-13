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
#include "BMP280.h"
#include "stdio.h"
#include "lcd.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* Variables for all measurements */
typedef struct{
	double temp , press;
} data;
/* USER CODE END Variables */
osThreadId thread_sensorHandle;
osThreadId thread_readHandle;
//osMessageQId tempHandle;
osMailQId mailHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void thread_sensor_fun(void const * argument);
void thread_read_func(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

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
	//double buffer;

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

  /* Create the queue(s) */
  /* mail messages */
  osMailQDef(mail, 16, data);
  //mailYPRHandle = osMailCreate(osMailQ(mailYPR), NULL);

  mailHandle = osMailCreate(osMailQ(mail), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of thread_sensor */
  osThreadDef(thread_sensor, thread_sensor_fun, osPriorityNormal, 0, 128);
  thread_sensorHandle = osThreadCreate(osThread(thread_sensor), NULL);

  /* definition and creation of thread_read */
  osThreadDef(thread_read, thread_read_func, osPriorityNormal, 0, 128);
  thread_readHandle = osThreadCreate(osThread(thread_read), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_thread_sensor_fun */
/**
  * @brief  Function implementing the thread_sensor thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_thread_sensor_fun */
void thread_sensor_fun(void const * argument)
{
  /* USER CODE BEGIN thread_sensor_fun */
    double temprature;

	int8_t com_rslt;

	bmp280_t bmp280;
	bmp280.i2c_handle = &hi2c1;
	bmp280.dev_addr = BMP280_I2C_ADDRESS1;
	com_rslt = BMP280_init(&bmp280);
	com_rslt += BMP280_set_power_mode(BMP280_NORMAL_MODE);
	com_rslt += BMP280_set_work_mode(BMP280_STANDARD_RESOLUTION_MODE);
	com_rslt += BMP280_set_standby_durn(BMP280_STANDBY_TIME_1_MS);
	if (com_rslt != SUCCESS) {
	        exit(0);
	}

	/* Read temperature and pressure */


	//BMP280_read_pressure_double(&press);

	/* Infinite loop */
  for(;;)
  {

	  data* buffer;
	  buffer = (data * )osMailAlloc(mailHandle,osWaitForever);
	  if (buffer != NULL){
	     buffer->temp = BMP280_read_temperature_double(&temprature);
         osMailPut(mailHandle,buffer);
	  }
    osDelay(1000);
  }
  /* USER CODE END thread_sensor_fun */
}

/* USER CODE BEGIN Header_thread_read_func */
/**
* @brief Function implementing the thread_read thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_thread_read_func */
void thread_read_func(void const * argument)
{
  /* USER CODE BEGIN thread_read_func */

	LCD_Init();
  /* Infinite loop */
	data* buffer;
	osEvent evt;
	double rx;
	for(;;)
  {
	  evt = osMailGet(mailHandle, osWaitForever);
	  if (evt.status == osEventMail){
	  rx = evt.value.v;
	  LCD_Printf("%0.4f sys",rx);
	  }
    osDelay(1);
  }
  /* USER CODE END thread_read_func */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
