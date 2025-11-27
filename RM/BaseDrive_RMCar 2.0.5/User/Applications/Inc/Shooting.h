#ifndef __SHOOTING_H
#define __SHOOTING_H

#include "main.h"
#include "PID.h"
#include "Remote_Control.h"
#include "My_Math.h"
#include "Motors.h"
#include "math.h"

//** 宏定义 **//
#define NEED_ACCUM_ANGEL 	1500.0f //6900.98f拨盘减速箱后计算值
#define DIAL_RED_ANGEL		6900.0f
/* 摩擦轮目标速度范围 */
#define FRICTION_RANGE		16384.0f
/* 定义摩擦轮电机的ID */
#define FRICTION_L 			1
#define FRICTION_R 			0

typedef enum
{
	SH_NOMAL_MOOD,
	SH_STOP_MOOD,
	SH_CHONTINUE_MOOD,
}SF_Mood_e;

typedef struct
{
	/**  拨盘 **/
	float Dial_TAngle;
	float Dial_CAngle;
	float Dial_LAngle;
	float Dial_AcumulateAngle;
	float Dial_ADDAngle;
	bool  Dial_IsF;//控制方向
	/**  摩擦轮 **/
	float Friction_TSpeed;
	float Friction_CLSpeed;
	float Friction_CRSpeed;
	/** 礼炮 **/
	float Salute_TAngle;
	float Salute_CAngle;
	float Salute_LAngle;
	float Salute_AcumulateAngle;
	float Salute_ADDAngle;
	bool  Salute_IsF;//控制方向
	
}Shooting_Variable_t;

void  Shooting_Init(void);
float Shooting_Dial(void);
void Shooting_Friction(SF_Mood_e SF_Mood);
void Shooting_Friction_Stop(void);
float Shooting_Salute(void);

#endif
