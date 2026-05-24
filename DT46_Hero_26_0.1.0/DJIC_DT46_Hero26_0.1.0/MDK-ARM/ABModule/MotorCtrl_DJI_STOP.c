#include "MotorCtrl_DJI_STOP.h"

void DJI_MOTOR_EmergencySTOP_ALL(PID_HandleTypeDef* Motor_STOP,const DJI_MotorFeedback_t DJI_MFeedback[],CAN_HandleTypeDef *hcan,float MError)
{
	int16_t PIDoutput1[4];
	int16_t PIDoutput2[4];
	
	DJI_MotorFeedback_t MFeedBack[8];
	
	for(int i = 0;i < 8;i++)
	{
		memcpy(&MFeedBack[i],&DJI_MFeedback[i], sizeof(DJI_MotorFeedback_t));
	}
	
	
	PIDoutput1[0] = PID_Calculate(Motor_STOP,MFeedBack[0].speed_rpm,0);
																
	PIDoutput1[1] = PID_Calculate(Motor_STOP,MFeedBack[1].speed_rpm,0);
	
	PIDoutput1[2] = PID_Calculate(Motor_STOP,MFeedBack[2].speed_rpm,0);
	
	PIDoutput1[3] = PID_Calculate(Motor_STOP,MFeedBack[3].speed_rpm,0);
	
	PIDoutput2[0] = PID_Calculate(Motor_STOP,MFeedBack[4].speed_rpm,0);
	
	PIDoutput2[1] = PID_Calculate(Motor_STOP,MFeedBack[5].speed_rpm,0);
	
	PIDoutput2[2] = PID_Calculate(Motor_STOP,MFeedBack[6].speed_rpm,0);
	
	PIDoutput2[3] = PID_Calculate(Motor_STOP,MFeedBack[7].speed_rpm,0);
	
	ESC_Control_Raw_Group(hcan,1,PIDoutput1);
	ESC_Control_Raw_Group(hcan,4,PIDoutput2);
}
