#include "Motor_Contral.h"

extern DJI_MotorFeedback_t DJI_MFeedback[8];

PID_HandleTypeDef test;


void Motor_CInit(void)
{
	PID_Init(&test,13.0f,0.2,1.0,-DJI_M2006_R,DJI_M2006_R,-500.0f, 500.0f);
}

void Motor_DJI_Speed_SingleContral(int16_t MotorVel)
{
	ESC_Control_Raw_Single(&hcan1,1,PID_Calculate(&test,DJI_MFeedback[0].speed_rpm,MotorVel));
}

void Motor_DJI_Speed_AngleContral(int16_t MotorVel)
{
	ESC_Control_Raw_Single(&hcan1,1,PID_Calculate(&test,DJI_MFeedback[0].speed_rpm,MotorVel));
}
