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
#include "MY_LED.h"
#include "DJI_Motor_CAN.h"
#include "Dual_board_Transmit.h"
#include "HERO_DriveSystem.h"

#include "User_CanCallBack.h"
#include "UAP_FreeRTOS.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
/* Definitions for HEROChassis */
osThreadId_t HEROChassisHandle;
uint32_t HEROChassisBuffer[ 128 ];
osStaticThreadDef_t HEROChassisControlBlock;
const osThreadAttr_t HEROChassis_attributes = {
  .name = "HEROChassis",
  .cb_mem = &HEROChassisControlBlock,
  .cb_size = sizeof(HEROChassisControlBlock),
  .stack_mem = &HEROChassisBuffer[0],
  .stack_size = sizeof(HEROChassisBuffer),
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for HERODial */
osThreadId_t HERODialHandle;
uint32_t HERODialBuffer[ 128 ];
osStaticThreadDef_t HERODialControlBlock;
const osThreadAttr_t HERODial_attributes = {
  .name = "HERODial",
  .cb_mem = &HERODialControlBlock,
  .cb_size = sizeof(HERODialControlBlock),
  .stack_mem = &HERODialBuffer[0],
  .stack_size = sizeof(HERODialBuffer),
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for HEROGimbal */
osThreadId_t HEROGimbalHandle;
uint32_t HEROGimbalBuffer[ 128 ];
osStaticThreadDef_t HEROGimbalControlBlock;
const osThreadAttr_t HEROGimbal_attributes = {
  .name = "HEROGimbal",
  .cb_mem = &HEROGimbalControlBlock,
  .cb_size = sizeof(HEROGimbalControlBlock),
  .stack_mem = &HEROGimbalBuffer[0],
  .stack_size = sizeof(HEROGimbalBuffer),
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for HEROShooting */
osThreadId_t HEROShootingHandle;
uint32_t HEROShootingBuffer[ 128 ];
osStaticThreadDef_t HEROShootingControlBlock;
const osThreadAttr_t HEROShooting_attributes = {
  .name = "HEROShooting",
  .cb_mem = &HEROShootingControlBlock,
  .cb_size = sizeof(HEROShootingControlBlock),
  .stack_mem = &HEROShootingBuffer[0],
  .stack_size = sizeof(HEROShootingBuffer),
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for HOTRCCtl */
osThreadId_t HOTRCCtlHandle;
uint32_t HOTRCCtlBuffer[ 128 ];
osStaticThreadDef_t HOTRCCtlControlBlock;
const osThreadAttr_t HOTRCCtl_attributes = {
  .name = "HOTRCCtl",
  .cb_mem = &HOTRCCtlControlBlock,
  .cb_size = sizeof(HOTRCCtlControlBlock),
  .stack_mem = &HOTRCCtlBuffer[0],
  .stack_size = sizeof(HOTRCCtlBuffer),
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for SBUSProcess */
osThreadId_t SBUSProcessHandle;
uint32_t SBUSProcessBuffer[ 128 ];
osStaticThreadDef_t SBUSProcessControlBlock;
const osThreadAttr_t SBUSProcess_attributes = {
  .name = "SBUSProcess",
  .cb_mem = &SBUSProcessControlBlock,
  .cb_size = sizeof(SBUSProcessControlBlock),
  .stack_mem = &SBUSProcessBuffer[0],
  .stack_size = sizeof(SBUSProcessBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void HEROChassisTask(void *argument);
void HERODialTask(void *argument);
void HEROGimbalTask(void *argument);
void HEROShootingTask(void *argument);
void HOTRCCtlTask(void *argument);
void SBUSProcessTask(void *argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of HEROChassis */
  HEROChassisHandle = osThreadNew(HEROChassisTask, NULL, &HEROChassis_attributes);

  /* creation of HERODial */
  HERODialHandle = osThreadNew(HERODialTask, NULL, &HERODial_attributes);

  /* creation of HEROGimbal */
  HEROGimbalHandle = osThreadNew(HEROGimbalTask, NULL, &HEROGimbal_attributes);

  /* creation of HEROShooting */
  HEROShootingHandle = osThreadNew(HEROShootingTask, NULL, &HEROShooting_attributes);

  /* creation of HOTRCCtl */
  HOTRCCtlHandle = osThreadNew(HOTRCCtlTask, NULL, &HOTRCCtl_attributes);

  /* creation of SBUSProcess */
  SBUSProcessHandle = osThreadNew(SBUSProcessTask, NULL, &SBUSProcess_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	UAP_FreeRTOS_Init();
	
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
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_HEROChassisTask */
/**
* @brief Function implementing the HEROChassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HEROChassisTask */
__weak void HEROChassisTask(void *argument)
{
  /* USER CODE BEGIN HEROChassisTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END HEROChassisTask */
}

/* USER CODE BEGIN Header_HERODialTask */
/**
* @brief Function implementing the HERODial thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HERODialTask */
__weak void HERODialTask(void *argument)
{
  /* USER CODE BEGIN HERODialTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END HERODialTask */
}

/* USER CODE BEGIN Header_HEROGimbalTask */
/**
* @brief Function implementing the HEROGimbal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HEROGimbalTask */
__weak void HEROGimbalTask(void *argument)
{
  /* USER CODE BEGIN HEROGimbalTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END HEROGimbalTask */
}

/* USER CODE BEGIN Header_HEROShootingTask */
/**
* @brief Function implementing the HEROShooting thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HEROShootingTask */
__weak void HEROShootingTask(void *argument)
{
  /* USER CODE BEGIN HEROShootingTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END HEROShootingTask */
}

/* USER CODE BEGIN Header_HOTRCCtlTask */
/**
* @brief Function implementing the HOTRCCtl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HOTRCCtlTask */
__weak void HOTRCCtlTask(void *argument)
{
  /* USER CODE BEGIN HOTRCCtlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END HOTRCCtlTask */
}

/* USER CODE BEGIN Header_SBUSProcessTask */
/**
* @brief Function implementing the SBUSProcess thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SBUSProcessTask */
__weak void SBUSProcessTask(void *argument)
{
  /* USER CODE BEGIN SBUSProcessTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SBUSProcessTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

