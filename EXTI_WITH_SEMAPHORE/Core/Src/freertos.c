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
osThreadId th1_ledHandle;
osThreadId th2_ledHandle;
osThreadId th3_sosHandle;
osSemaphoreId bt_Handle;
osMutexId led_Handle;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void th1_led_fun(void const * argument);
void th2_led_fun(void const * argument);
void th3_sos_fun(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
	osMutexDef(mutex_led);
    led_Handle = osMutexCreate(osMutex(mutex_led));
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of semaphore_bt */
  osSemaphoreDef(semaphore_bt);
  bt_Handle = osSemaphoreCreate(osSemaphore(semaphore_bt), 1);

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
  /* definition and creation of th1_led */
  osThreadDef(th1_led, th1_led_fun, osPriorityAboveNormal, 0, 128);
  th1_ledHandle = osThreadCreate(osThread(th1_led), NULL);

  /* definition and creation of th2_led */
  osThreadDef(th2_led, th2_led_fun, osPriorityNormal, 0, 128);
  th2_ledHandle = osThreadCreate(osThread(th2_led), NULL);

  /* definition and creation of th3_sos */
  osThreadDef(th3_sos, th3_sos_fun, osPriorityHigh, 0, 128);
  th3_sosHandle = osThreadCreate(osThread(th3_sos), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_th1_led_fun */
/**
* @brief Function implementing the th1_led thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_th1_led_fun */
void th1_led_fun(void const * argument)
{
  /* USER CODE BEGIN th1_led_fun */
  /* Infinite loop */
  for(;;)
  {
	if(osMutexWait(led_Handle,osWaitForever)==osOK){
		uint32_t i = 15;
		while(i--){
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		  osDelay(700);
		}
		osMutexRelease(led_Handle);
	}
   osDelay(1);
  }
  /* USER CODE END th1_led_fun */
}

/* USER CODE BEGIN Header_th2_led_fun */
/**
* @brief Function implementing the th2_led thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_th2_led_fun */
void th2_led_fun(void const * argument)
{
  /* USER CODE BEGIN th2_led_fun */
  /* Infinite loop */
	  for(;;)
	    {
	  	if(osMutexWait(led_Handle,osWaitForever)==osOK){
	  		uint32_t i = 15;
	  		while(i--){
	  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  		  osDelay(1000);
	  		}
	  		osMutexRelease(led_Handle);
	  	}


       }
	  osDelay(1);
  /* USER CODE END th2_led_fun */
}

/* USER CODE BEGIN Header_th3_sos_fun */
/**
* @brief Function implementing the th3_sos thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_th3_sos_fun */
void th3_sos_fun(void const * argument)
{
  /* USER CODE BEGIN th3_sos_fun */
   osSemaphoreWait(bt_Handle,0);
	/* Infinite loop */
  for(;;)
  {
   if(osSemaphoreWait(bt_Handle, osWaitForever) == osOK){
   uint32_t i = 100;
   while(i--){
	   HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	   osDelay(100);
    }
   }
   osDelay(1);
  }
  /* USER CODE END th3_sos_fun */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == B1_Pin) {
		osSemaphoreRelease(bt_Handle);
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
