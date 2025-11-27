/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include "stdio.h"
#include "Remote_Control.h"
#include "Motors.h"
#include "bsp_can.h"
#include "INS_task.h"
#include "Dual_board_Transmit.h"
#include "PID.h"
#include "ReSet_ToInit.h"
#include "INS_task.h"
#include "Gimbal_Move.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//extern RC_Ctl_t RC_Ctl;
//extern motor_measure_t motor_chassis[7];
//extern PID_HandleTypeDef GimbalPID_Yaw_Angle;
//extern PID_HandleTypeDef GimbalPID_Yaw_Speed;
//extern PID_HandleTypeDef GimbalPID_Pitch_Motor;
//extern PID_HandleTypeDef GimbalPID_Shoot_Motor;
//
//extern float RxInitYawFlag;
//extern float InitYawFlag;
//
//extern float IMU_Yaw;
//float InitFlag = 0;
//extern float RxInitYawFlag;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//uint8_t num = 0;
//uint8_t EnterFlag = 0;
//void Get_Rate(void)
//{
//	if(EnterFlag == 0)
//	{
//		if((motor_chassis[6].Degree_Angle <= 360.0f && motor_chassis[6].Degree_Angle >= 350.0f))
//		{
//			EnterFlag = 1;
//			num = num + 1;
//		}
//	
//	}
//	else if(EnterFlag == 1)
//	{
//		if((motor_chassis[6].Degree_Angle <= 250.0f && motor_chassis[6].Degree_Angle >= 200.0f))
//		{
//			EnterFlag = 0;
//		}
//	}
//}


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
/* Definitions for LEDLightTask */
osThreadId_t LEDLightTaskHandle;
const osThreadAttr_t LEDLightTask_attributes = {
  .name = "LEDLightTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

///* Definitions for imuTask */
//osThreadId_t imuTaskHandle;
//const osThreadAttr_t imuTask_attributes = {
//  .name = "imuTask",
//  .stack_size = 256 * 4,
//  .priority = (osPriority_t) osPriorityRealtime1,
//};


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);

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

  /* creation of LEDLightTask */
  LEDLightTaskHandle = osThreadNew(StartTask02, NULL, &LEDLightTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(INS_task, NULL, &imuTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
    //imuTaskHandle = osThreadNew(INS_task, NULL, &imuTask_attributes);

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
float TempTest;
//extern double get_accumulated_angle(double current_angle);
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	ReSet_LEDTip();
  for(;;)
  {	 
	  
	  GetIMU_Value();
	  GetReSetFlag();
	  ReSet_ToInit();
	  
	 //printf("%f,%f,%f,%f,%f,%f\n",GimbalPID_Pitch_Motor.Target,GimbalPID_Pitch_Motor.Current,GimbalPID_Pitch_Motor.Output,GimbalPID_Pitch_Motor.Kp,GimbalPID_Pitch_Motor.Ki,GimbalPID_Pitch_Motor.Kd);
	 //printf("%f,%f,%f,%f,%f,%f\n",GimbalPID_Yaw_Motor.Target,GimbalPID_Yaw_Motor.Current,GimbalPID_Yaw_Motor.Output,GimbalPID_Yaw_Motor.Kp,GimbalPID_Yaw_Motor.Ki,GimbalPID_Yaw_Motor.Kd);
	 //printf("%f,%f,%f,%f,%f,%f\n",ChassisPID_Motor.Target,Speed,ChassisPID_Motor.Output,ChassisPID_Motor.Kp,ChassisPID_Motor.Ki,ChassisPID_Motor.Kd);
	 //printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",GimbalPID_Yaw_Angle.Target,GimbalPID_Yaw_Angle.Current,GimbalPID_Yaw_Angle.Output,GimbalPID_Yaw_Angle.Kp,GimbalPID_Yaw_Angle.Ki,GimbalPID_Yaw_Angle.Kd,GimbalPID_Yaw_Speed.Target,GimbalPID_Yaw_Speed.Current,GimbalPID_Yaw_Speed.Output,GimbalPID_Yaw_Speed.Kp,GimbalPID_Yaw_Speed.Ki,GimbalPID_Yaw_Speed.Kd); 
	  osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the LEDLightTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
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

