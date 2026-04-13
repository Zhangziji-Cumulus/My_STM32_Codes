#include "Motor_Contral.h"

extern DJI_MotorFeedback_t DJI_MFeedback[8];

PID_FF_HandleTypeDef test_FF;
PID_HandleTypeDef test2;
PID_HandleTypeDef test3;
void Motor_CInit(void)
{
	PID_FF_Init(&test_FF,0.0f,0.0f,0.0f,100.0,-DJI_M2006_R,DJI_M2006_R,-500.0f, 500.0f);
	PID_Init(&test2,15.0f,0.0f,0.0f,-DJI_M2006_R,DJI_M2006_R,-500.0f, 500.0f);
	PID_Init(&test3,20.0f,0.0f,0.0f,-DJI_M2006_R,DJI_M2006_R,-500.0f, 500.0f);
}

void Motor_DJI_Speed_SingleContral(int16_t MotorVel)
{
	int16_t PIDoutput = PID_FF_Calculate_AutoFF(&test_FF,DJI_MFeedback[0].speed_rpm,MotorVel,PID_FF_MODE_VELOCITY);
	
	ESC_Control_Raw_Single(&hcan1,1,PIDoutput);
}

float Angletest = 0.0;

void Motor_DJI_Angle_SingleContral(float TargetAngle)
{
	TargetAngle = MyMath_normalize_m180_to_p180(TargetAngle);	
	Angletest = MyMath_cal_output_angle(DJI_MFeedback[0].angle_deg,36);
	
	//int16_t PIDoutput = PID_Double_CycleAngle(&test2,&test3,TargetAngle,DJI_MFeedback[0].speed_rpm,Angletest,1.00f);
	int16_t PIDoutput = PID_FF_Calculate_CycleAngle(&test_FF,Angletest,TargetAngle);
	//int16_t PIDoutput = PID_Calculate_CycleAngle(&test2,Angletest,TargetAngle);

	ESC_Control_Raw_Single(&hcan1,1,PIDoutput);
}



