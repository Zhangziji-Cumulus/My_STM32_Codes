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
#include "Dual_board_Transmit.h"
#include "HERO_DriveSystem.h"

#include "User_CanCallBack.h"
#include "UAP_FreeRTOS.h"

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
  .priority = (osPriority_t) osPriorityNormal,
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
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for buzr */
osThreadId_t buzrHandle;
const osThreadAttr_t buzr_attributes = {
  .name = "buzr",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DualBoard */
osThreadId_t DualBoardHandle;
uint32_t Dual_Board_TranBuffer[ 256 ];
osStaticThreadDef_t Dual_Board_TranControlBlock;
const osThreadAttr_t DualBoard_attributes = {
  .name = "DualBoard",
  .cb_mem = &Dual_Board_TranControlBlock,
  .cb_size = sizeof(Dual_Board_TranControlBlock),
  .stack_mem = &Dual_Board_TranBuffer[0],
  .stack_size = sizeof(Dual_Board_TranBuffer),
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for HEROChassis */
osThreadId_t HEROChassisHandle;
uint32_t HERO_DriveSysteBuffer[ 128 ];
osStaticThreadDef_t HERO_DriveSysteControlBlock;
const osThreadAttr_t HEROChassis_attributes = {
  .name = "HEROChassis",
  .cb_mem = &HERO_DriveSysteControlBlock,
  .cb_size = sizeof(HERO_DriveSysteControlBlock),
  .stack_mem = &HERO_DriveSysteBuffer[0],
  .stack_size = sizeof(HERO_DriveSysteBuffer),
  .priority = (osPriority_t) osPriorityHigh7,
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
/* Definitions for HEROSystem */
osThreadId_t HEROSystemHandle;
uint32_t HEROSystemBuffer[ 128 ];
osStaticThreadDef_t HEROSystemControlBlock;
const osThreadAttr_t HEROSystem_attributes = {
  .name = "HEROSystem",
  .cb_mem = &HEROSystemControlBlock,
  .cb_size = sizeof(HEROSystemControlBlock),
  .stack_mem = &HEROSystemBuffer[0],
  .stack_size = sizeof(HEROSystemBuffer),
  .priority = (osPriority_t) osPriorityHigh7,
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
  .priority = (osPriority_t) osPriorityHigh7,
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
/* Definitions for g_CAN2_Queue */
osMessageQueueId_t g_CAN2_QueueHandle;
uint8_t g_CAN2_QueueBuffer[ 16 * sizeof( uint16_t ) ];
osStaticMessageQDef_t g_CAN2_QueueControlBlock;
const osMessageQueueAttr_t g_CAN2_Queue_attributes = {
  .name = "g_CAN2_Queue",
  .cb_mem = &g_CAN2_QueueControlBlock,
  .cb_size = sizeof(g_CAN2_QueueControlBlock),
  .mq_mem = &g_CAN2_QueueBuffer,
  .mq_size = sizeof(g_CAN2_QueueBuffer)
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartRealTime_TASK(void *argument);
void Start_DJI_RecieveData(void *argument);
void Dual_Board_Transmit_Task(void *argument);
void HEROChassisTask(void *argument);
void HEROGimbalTask(void *argument);
void HEROShootingTask(void *argument);
void HEROSystemTask(void *argument);
void HERODialTask(void *argument);

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

  /* creation of g_CAN2_Queue */
  g_CAN2_QueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &g_CAN2_Queue_attributes);

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

  /* creation of DualBoard */
  DualBoardHandle = osThreadNew(Dual_Board_Transmit_Task, NULL, &DualBoard_attributes);

  /* creation of HEROChassis */
  HEROChassisHandle = osThreadNew(HEROChassisTask, NULL, &HEROChassis_attributes);

  /* creation of HEROGimbal */
  HEROGimbalHandle = osThreadNew(HEROGimbalTask, NULL, &HEROGimbal_attributes);

  /* creation of HEROShooting */
  HEROShootingHandle = osThreadNew(HEROShootingTask, NULL, &HEROShooting_attributes);

  /* creation of HEROSystem */
  HEROSystemHandle = osThreadNew(HEROSystemTask, NULL, &HEROSystem_attributes);

  /* creation of HERODial */
  HERODialHandle = osThreadNew(HERODialTask, NULL, &HERODial_attributes);

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
	
//	static uint16_t timecounttest = 0;
//	buzzer_t *buzzer = get_buzzer_effect_point();
//	buzzer->work = FALSE;
//	buzzer->work = TRUE;
	
  for(;;)
  {
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

/* USER CODE BEGIN Header_buzzer_effects_task */
/**
* @brief Function implementing the buzr thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_buzzer_effects_task */

/* USER CODE BEGIN Header_Dual_Board_Transmit_Task */
/**
* @brief Function implementing the Dual_Board_Tran thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Dual_Board_Transmit_Task */
__weak void Dual_Board_Transmit_Task(void *argument)
{
  /* USER CODE BEGIN Dual_Board_Transmit_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Dual_Board_Transmit_Task */
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

/* USER CODE BEGIN Header_HEROSystemTask */
/**
* @brief Function implementing the HEROSystem thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HEROSystemTask */
__weak void HEROSystemTask(void *argument)
{
  /* USER CODE BEGIN HEROSystemTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END HEROSystemTask */
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

