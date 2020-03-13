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
#include "BMP280.h"
#include "i2c.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "lcd.h"

/* USER CODE BEGIN Includes */     
typedef struct {

	double temp,pres;

} var;

//volatile uint8_t buf;

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

/* USER CODE END Variables */
osThreadId thread_sensorHandle;
osThreadId thread_uartHandle;
osMailQId pipeHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void thread_sensor_func(void const * argument);
void thread_uart_func(void const * argument);

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

  /* Create the queue(s) */
  /* definition and creation of pipe */
  osMailQDef(pipe, 16, uint16_t);
  pipeHandle = osMailCreate(osMailQ(pipe), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of thread_sensor */
  osThreadDef(thread_sensor, thread_sensor_func, osPriorityHigh, 0, 128);
  thread_sensorHandle = osThreadCreate(osThread(thread_sensor), NULL);

  /* definition and creation of thread_uart */
  osThreadDef(thread_uart, thread_uart_func, osPriorityNormal, 0, 128);
  thread_uartHandle = osThreadCreate(osThread(thread_uart), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_thread_sensor_func */
/**
  * @brief  Function implementing the thread_sensor thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_thread_sensor_func */
void thread_sensor_func(void const * argument)
{
  /* USER CODE BEGIN thread_sensor_func */
	/* Variables for all measurements */
	double d1 ,d2;
	int8_t com_rslt;

	/* Start BMP280 and change settings */
	//LCD_Printf("Connecting to BMP280...\n");
	bmp280_t bmp280;
	bmp280.i2c_handle = &hi2c1;
	bmp280.dev_addr = BMP280_I2C_ADDRESS1;
	com_rslt = BMP280_init(&bmp280);
	com_rslt += BMP280_set_power_mode(BMP280_NORMAL_MODE);
	com_rslt += BMP280_set_work_mode(BMP280_STANDARD_RESOLUTION_MODE);
	com_rslt += BMP280_set_standby_durn(BMP280_STANDBY_TIME_1_MS);

	if (com_rslt != SUCCESS) {


		    LCD_Printf("Check BMP280 connection!\nProgram terminated");
	        exit(0);
	}

	LCD_Printf("Connection successful!\n");


	/* Infinite loop */
  for(;;)
  {
	  /* Read temperature and pressure */

	  /*pipe line start*/
	  	 var * flow;
	  	 flow = (var * )osMailAlloc(pipeHandle,osWaitForever);
	  	 if (flow != NULL){
	  		 /*read to send data */
	  		 flow->temp = BMP280_read_temperature_double(&d1);
	  		 flow->pres = BMP280_read_pressure_double(&d2);
	  		 osMailPut(pipeHandle,flow);
	  	 }
    osDelay(5000);
  }
  /* USER CODE END thread_sensor_func */
}

/* USER CODE BEGIN Header_thread_uart_func */
/**
* @brief Function implementing the thread_uart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_thread_uart_func */
void thread_uart_func(void const * argument)
{
  /* USER CODE BEGIN thread_uart_func */
  /* Infinite loop */
	for(;;)
  {
	  /* pipe end */
      var * flow;
      osEvent  ev;
      ev = osMailGet(pipeHandle,osWaitForever);
      //data verify
      //uint8_t text[] = " UART IS ON \n";
      //HAL_UART_Transmit(&huart2, text, sizeof(text), HAL_MAX_DELAY);
      if (ev.status == osEventMail){
    	  //uart_verify
    	  //getting
    	  flow = (var *) ev.value.p;
    	  char  buf[512];
    	  double get_d1 =  flow->temp;
    	  char rx = snprintf(buf, 100 ,"the temprature  is %0.4f" ,get_d1);
    	  HAL_UART_Transmit(&huart2,  rx , sizeof(rx), HAL_MAX_DELAY);
    	  osMailFree(pipeHandle, flow);
      }
    osDelay(100);
  }
  /* USER CODE END thread_uart_func */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
