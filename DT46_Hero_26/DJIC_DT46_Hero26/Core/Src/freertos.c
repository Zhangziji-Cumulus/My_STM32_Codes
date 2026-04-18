/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "INS_task.h"
#include "MY_LED.h"
#include "DJI_Motor_CAN.h"
#include "sound_effects_task.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RealTime_TASK */
osThreadId_t RealTime_TASKHandle;
uint32_t RealTime_TASKBuffer[ 128 ];
osStaticThreadDef_t RealTime_TASKControlBlock;
const osThreadAttr_t RealTime_TASK_attributes = {
  .name = "RealTime_TASK",
  .cb_mem = &RealTime_TASKControlBlock,
  .cb_size = sizeof(RealTime_TASKControlBlock),
  .stack_mem = &RealTime_TASKBuffer[0],
  .stack_size = sizeof(RealTime_TASKBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for DJI_RecieveData */
osThreadId_t DJI_RecieveDataHandle;
uint32_t DJI_RecieveDataBuffer[ 128 ];
osStaticThreadDef_t DJI_RecieveDataControlBlock;
const osThreadAttr_t DJI_RecieveData_attributes = {
  .name = "DJI_RecieveData",
  .cb_mem = &DJI_RecieveDataControlBlock,
  .cb_size = sizeof(DJI_RecieveDataControlBlock),
  .stack_mem = &DJI_RecieveDataBuffer[0],
  .stack_size = sizeof(DJI_RecieveDataBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for buzr */
osThreadId_t buzrHandle;
const osThreadAttr_t buzr_attributes = {
  .name = "buzr",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Queue_DJI_MD */
osMessageQueueId_t Queue_DJI_MDHandle;
uint8_t myQueue01testBuffer[ 5 * sizeof( uint16_t ) ];
osStaticMessageQDef_t myQueue01testControlBlock;
const osMessageQueueAttr_t Queue_DJI_MD_attributes = {
  .name = "Queue_DJI_MD",
  .cb_mem = &myQueue01testControlBlock,
  .cb_size = sizeof(myQueue01testControlBlock),
  .mq_mem = &myQueue01testBuffer,
  .mq_size = sizeof(myQueue01testBuffer)
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartRealTime_TASK(void *argument);
void Start_DJI_RecieveData(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of Queue_DJI_MD */
  Queue_DJI_MDHandle = osMessageQueueNew (5, sizeof(uint16_t), &Queue_DJI_MD_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of RealTime_TASK */
  RealTime_TASKHandle = osThreadNew(StartRealTime_TASK, NULL, &RealTime_TASK_attributes);

  /* creation of DJI_RecieveData */
  DJI_RecieveDataHandle = osThreadNew(Start_DJI_RecieveData, NULL, &DJI_RecieveData_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(INS_task, NULL, &imuTask_attributes);

  /* creation of buzr */
  buzrHandle = osThreadNew(buzzer_effects_task, NULL, &buzr_attributes);

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
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	
//	static uint16_t timecounttest = 0;
//	buzzer_t *buzzer = get_buzzer_effect_point();
//	buzzer->work = FALSE;
//	buzzer->work = TRUE;
	
  for(;;)
  {
	  //buzzer->sound_effect = D_D_D_;
		
		//HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_B_Pin,LED_Flash(500,1));
		//HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_G_Pin,LED_Flash(800,1));
		//HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,LED_Flash(800,1));
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartRealTime_TASK */
/**
* @brief Function implementing the RealTime_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRealTime_TASK */
__weak void StartRealTime_TASK(void *argument)
{
  /* USER CODE BEGIN StartRealTime_TASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRealTime_TASK */
}

/* USER CODE BEGIN Header_Start_DJI_RecieveData */
/**
* @brief Function implementing the DJI_RecieveData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_DJI_RecieveData */
__weak void Start_DJI_RecieveData(void *argument)
{
  /* USER CODE BEGIN Start_DJI_RecieveData */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_DJI_RecieveData */
}

/* USER CODE BEGIN Header_INS_task */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_task */


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

