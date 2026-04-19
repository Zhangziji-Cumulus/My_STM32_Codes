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
  .stack_size = 256 * 4,
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
/* Definitions for HERODriveSystem */
osThreadId_t HERODriveSystemHandle;
uint32_t HERO_DriveSysteBuffer[ 128 ];
osStaticThreadDef_t HERO_DriveSysteControlBlock;
const osThreadAttr_t HERODriveSystem_attributes = {
  .name = "HERODriveSystem",
  .cb_mem = &HERO_DriveSysteControlBlock,
  .cb_size = sizeof(HERO_DriveSysteControlBlock),
  .stack_mem = &HERO_DriveSysteBuffer[0],
  .stack_size = sizeof(HERO_DriveSysteBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
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
uint8_t g_CAN2_QueueBuffer[ 16 * sizeof( CAN2_RxMsg_t ) ];
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
void HERODriveSystemTask(void *argument);

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
  g_CAN2_QueueHandle = osMessageQueueNew (16, sizeof(CAN2_RxMsg_t), &g_CAN2_Queue_attributes);

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

  /* creation of HERODriveSystem */
  HERODriveSystemHandle = osThreadNew(HERODriveSystemTask, NULL, &HERODriveSystem_attributes);

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

/* USER CODE BEGIN Header_HERODriveSystemTask */
/**
* @brief Function implementing the HERODriveSystem thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HERODriveSystemTask */
__weak void HERODriveSystemTask(void *argument)
{
  /* USER CODE BEGIN HERODriveSystemTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END HERODriveSystemTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

