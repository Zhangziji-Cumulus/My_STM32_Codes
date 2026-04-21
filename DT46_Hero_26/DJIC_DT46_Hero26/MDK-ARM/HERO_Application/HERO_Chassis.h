#ifndef __HERO_CHASSIS_H
#define __HERO_CHASSIS_H

#include "HERO_API.h"

typedef struct{

	int16_t Xv;// 前后
  int16_t Yv;// 左右
  int16_t Rv;// 旋转
	
}Mecanum_Target_t;

//每个轮子单独的数据
typedef struct{

	int16_t T_Speed;     //目标速度
	int16_t C_Speed;     //当前速度
	int16_t C_Curr;    //当前电流
	int16_t C_Enc;     //当前编码器值
	int16_t Out_vel;   //输出值

}Wheel_Data_t;

//四个轮子的数据
typedef struct{

	Wheel_Data_t W_FL;
	Wheel_Data_t W_FR;
	Wheel_Data_t W_BL;
	Wheel_Data_t W_BR;
	
}Wheels_Data_t;

//麦轮解算
typedef struct{
	
	Mecanum_Target_t Target;
	
	Wheel_Data_t FL;
	Wheel_Data_t FR;
	Wheel_Data_t BL;
	Wheel_Data_t BR;
	
}Mecanum_Data_t;

//每个麦轮速度解算
void Chassis_Mecanum_Calc(Mecanum_Data_t* pMD);

#endif
