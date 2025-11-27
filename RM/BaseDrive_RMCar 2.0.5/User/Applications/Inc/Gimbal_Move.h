#ifndef __GIMBAL_MOVE_H
#define __GIMBAL_MOVE_H

#include "Chassis_Move.h"
#include "Remote_Control.h"
#include "Motors.h"
#include "PID.h"
#include "stdio.h"
#include "INS_task.h"
#include "Friction_Wheel_Shooting.h"
#include "My_Math.h"

//** 云台宏定义 **//
/* Yaw轴云台增量范围 */
#define YAW_ADDVALUE 0.10f
/* Yaw允许最大误差 */
#define YAW_ERROR 0.10f
/* Pitch轴云台增量范围 */
#define PITCH_ADDVALUE 0.15f
/* Pitch允许最大误差 */
#define PITCH_ERROR 0.10f
/* 限制Pitch轴角度范围 */
#define PITCH_RANGE_MIN 160.00f
#define PITCH_RANGE_MAX 210.00f
typedef struct 
{
	float Yaw;
	float Pitch;	
}Gimbal_Target_Direction;

typedef struct
{
	uint8_t Yaw_InitFlag;//Yaw轴初始化
	float YawTheta;
}GimBal_Variable_t;

void GimBal_Yaw_Motor_Set(float Dial,float Salute);
float GimBal_Pitch_Motor_Set(void);
void Gimbal_Init(void);

#endif
