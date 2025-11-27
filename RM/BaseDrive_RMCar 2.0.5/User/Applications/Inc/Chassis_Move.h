#ifndef __CHASSIS_MOVE_H
#define __CHASSIS_MOVE_H

#include "Chassis_Move.h"
#include "Remote_Control.h"
#include "Motors.h"
#include "My_Math.h"
#include "PID.h"
//** 宏定义 **//


/*目标速度比例*/
#define SPEED_RATIO	6.0f	

//** 底盘电机宏定义 **//

/* 电机数据序号 */
#define MOTOR_F_L 1
#define MOTOR_F_R 0
#define MOTOR_B_L 2
#define MOTOR_B_R 3
/*电机允许最小误差*/
#define ERROR_MOTOR_CH 		1.0f//底盘电机允许最小误差
#define ERROR_FOLLOWING 	0.1f
/*定义编码器零位偏移值*/
#define ENCODER_ZERO_OFFSET 344.575f
/*定义小陀螺旋转角速度*/
#define ROTATION_VALUE		3000.00f

//** 结构体定义 **//

typedef struct//目标方向
{
	float FB_Vel;
	float RL_Vel;
	float Rotation_Vel;
	
}Target_Direction_Vel;

typedef struct//底盘四个轮子的速度
{
	float F_L_Vel;
	float F_R_Vel;
	float B_L_Vel;
	float B_R_Vel;
	
}Chassis_Motor_Vel;

typedef struct
{

	float RelativeAngle_Radian;
	float RelativeAngle_Degree;
	float Theta_Radian;
	float Theta_Degree;
	
	float Following_CSpeed;
	
}Chassis_Variable;

void Chassis_Motor_Set(void);
void Chassis_Init(void);
	
#endif
