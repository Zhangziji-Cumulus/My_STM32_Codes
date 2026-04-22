#include "DJI_Motor_Contral.h"

extern DJI_MotorFeedback_t DJI_MFeedback[8];


PID_HandleTypeDef test2;
PID_HandleTypeDef test3;

PID_HandleTypeDef Chassis_Motor_In;
PID_HandleTypeDef Chassis_Motor_Ex;
PID_HandleTypeDef Chassis_Follow_PID;

PID_FF_HandleTypeDef Gimbal_Yaw_FF;
PID_HandleTypeDef Gimbal_Yaw_In;
PID_HandleTypeDef Gimbal_Yaw_Ex;

PID_FF_HandleTypeDef Gimbal_Pitch_FF;
PID_HandleTypeDef Gimbal_Pitch_In;
PID_HandleTypeDef Gimbal_Pitch_Ex;



void Motor_Init(void)
{
	PID_Init(&Chassis_Motor_In,3.0f,0.0f,0.0f,-DJI_M2006_R,DJI_M2006_R,-500.0f, 500.0f);
	PID_Init(&Chassis_Motor_Ex,3.0f,0.0f,0.0f,-DJI_M2006_R,DJI_M2006_R,-500.0f, 500.0f);
	PID_Init(&Chassis_Follow_PID,3.0f,0.0f,0.0f,-DJI_M2006_R,DJI_M2006_R,-500.0f, 500.0f);
	
	PID_FF_Init(&Gimbal_Yaw_FF,3.0f,0.0f,0.0f,1.0,-DJI_M2006_R,DJI_M2006_R,-500.0f, 500.0f);
	PID_Init(&Gimbal_Yaw_In,1.0f,0.0f,0.0f,-DJI_M2006_R,DJI_M2006_R,-500.0f, 500.0f);
	PID_Init(&Gimbal_Yaw_Ex,200.0f,0.0f,0.0f,-DJI_M2006_R,DJI_M2006_R,-500.0f, 500.0f);
	
	PID_FF_Init(&Gimbal_Pitch_FF,3.0f,0.0f,0.0f,1.0,-DJI_M2006_R,DJI_M2006_R,-500.0f, 500.0f);
	PID_Init(&Gimbal_Pitch_In,1.0f,0.0f,0.0f,-DJI_M2006_R,DJI_M2006_R,-500.0f, 500.0f);
	PID_Init(&Gimbal_Pitch_Ex,200.0f,0.0f,0.0f,-DJI_M2006_R,DJI_M2006_R,-500.0f, 500.0f);
	
}

void Motor_DJI_Speed_SingleContral(int16_t MotorVel)
{
	//int16_t PIDoutput = PID_FF_Calculate_AutoFF(&test_FF,DJI_MFeedback[0].speed_rpm,MotorVel,PID_FF_MODE_VELOCITY);
	//ESC_Control_Raw_Single(&hcan1,1,PIDoutput);
}

void Motor_DJI_SpeedCtl_1_4(CAN_HandleTypeDef *hcan,
	                          float   MError,
														int16_t MotorVel_1,
														int16_t MotorVel_2,
														int16_t MotorVel_3,
														int16_t MotorVel_4){
					
	int16_t PIDoutput[4];	
																
	PIDoutput[0] = PID_Double_Calculate(&Chassis_Motor_In,
															 &Chassis_Motor_Ex, 
															 MotorVel_1,
															 DJI_MFeedback[0].current_ma,
															 DJI_MFeedback[0].speed_rpm,
															 MError);
																
	PIDoutput[1] = PID_Double_Calculate(&Chassis_Motor_In,
															 &Chassis_Motor_Ex, 
															 MotorVel_1,
															 DJI_MFeedback[1].current_ma,
															 DJI_MFeedback[1].speed_rpm,
															 MError);
																
  PIDoutput[2] = PID_Double_Calculate(&Chassis_Motor_In,
															 &Chassis_Motor_Ex, 
															 MotorVel_1,
															 DJI_MFeedback[2].current_ma,
															 DJI_MFeedback[2].speed_rpm,
															 MError);

  PIDoutput[3] = PID_Double_Calculate(&Chassis_Motor_In,
															 &Chassis_Motor_Ex, 
															 MotorVel_1,
															 DJI_MFeedback[3].current_ma,
															 DJI_MFeedback[3].speed_rpm,
															 MError);

	ESC_Control_Raw_Group(&hcan1,1,PIDoutput);
}
															
void Motor_DJI_SpeedCtl_5_8(CAN_HandleTypeDef *hcan,
	                          float   MError,
														int16_t MotorVel_1,
														int16_t MotorVel_2,
														int16_t MotorVel_3,
														int16_t MotorVel_4){
					
	int16_t PIDoutput[4];	
																
	PIDoutput[0] = PID_Double_Calculate(&Chassis_Motor_In,
																			&Chassis_Motor_Ex, 
																			MotorVel_1,
																			DJI_MFeedback[4].current_ma,
																			DJI_MFeedback[4].speed_rpm,
																			MError);
																
	PIDoutput[1] = PID_Double_Calculate(&Chassis_Motor_In,
																			&Chassis_Motor_Ex, 
																			MotorVel_1,
																			DJI_MFeedback[5].current_ma,
																			DJI_MFeedback[5].speed_rpm,
																			MError);
																
  PIDoutput[2] = PID_Double_Calculate(&Chassis_Motor_In,
																			&Chassis_Motor_Ex, 
																			MotorVel_1,
																		  DJI_MFeedback[6].current_ma,
																		  DJI_MFeedback[6].speed_rpm,
																		  MError);

  PIDoutput[3] = PID_Double_Calculate(&Chassis_Motor_In,
																			&Chassis_Motor_Ex, 
																			MotorVel_1,
																			DJI_MFeedback[7].current_ma,
																			DJI_MFeedback[7].speed_rpm,
																			MError);

	ESC_Control_Raw_Group(hcan,5,PIDoutput);
}

void Motor_DJI_Speed_8Contral(CAN_HandleTypeDef *hcan,
															float   MError,
	                            int16_t MotorVel_1,
															int16_t MotorVel_2,
															int16_t MotorVel_3,
															int16_t MotorVel_4,
															int16_t MotorVel_5,
															int16_t MotorVel_6,
															int16_t MotorVel_7,
															int16_t MotorVel_8){
																
	int16_t PIDoutput[8];	
																
	PIDoutput[0] = PID_Double_Calculate(&Chassis_Motor_In,
																			&Chassis_Motor_Ex, 
																		  MotorVel_1,
																			DJI_MFeedback[0].current_ma,
																			DJI_MFeedback[0].speed_rpm,
																			MError);
																
	PIDoutput[1] = PID_Double_Calculate(&Chassis_Motor_In,
																			&Chassis_Motor_Ex, 
																			MotorVel_1,
																			DJI_MFeedback[1].current_ma,
																			DJI_MFeedback[1].speed_rpm,
																			MError);
																
  PIDoutput[2] = PID_Double_Calculate(&Chassis_Motor_In,
																			&Chassis_Motor_Ex, 
																			MotorVel_1,
																			DJI_MFeedback[2].current_ma,
																			DJI_MFeedback[2].speed_rpm,
																			MError);

  PIDoutput[3] = PID_Double_Calculate(&Chassis_Motor_In,
																			&Chassis_Motor_Ex, 
																			MotorVel_1,
																			DJI_MFeedback[3].current_ma,
																			DJI_MFeedback[3].speed_rpm,
																			MError);
																
	PIDoutput[4] = PID_Double_Calculate(&Chassis_Motor_In,
																			&Chassis_Motor_Ex, 
																			MotorVel_1,
																			DJI_MFeedback[4].current_ma,
																			DJI_MFeedback[4].speed_rpm,
																			MError);
																
	PIDoutput[5] = PID_Double_Calculate(&Chassis_Motor_In,
																			&Chassis_Motor_Ex, 
																			MotorVel_1,
																			DJI_MFeedback[5].current_ma,
																			DJI_MFeedback[5].speed_rpm,
																			MError);
																
  PIDoutput[6] = PID_Double_Calculate(&Chassis_Motor_In,
																			&Chassis_Motor_Ex, 
																			MotorVel_1,
																		  DJI_MFeedback[6].current_ma,
																		  DJI_MFeedback[6].speed_rpm,
																		  MError);

  PIDoutput[7] = PID_Double_Calculate(&Chassis_Motor_In,
																			&Chassis_Motor_Ex, 
																			MotorVel_1,
																			DJI_MFeedback[7].current_ma,
																			DJI_MFeedback[7].speed_rpm,
																			MError);
	
	ESC_Control_Raw_All(hcan,PIDoutput);
}




float Angletest = 0.0;

void Motor_DJI_Angle_SingleContral(float TargetAngle,uint8_t ID,uint16_t gear_ratio)
{
	TargetAngle = MyMath_normalize_m180_to_p180(TargetAngle);	
	Angletest = MyMath_cal_output_angle(DJI_MFeedback[ID - 1].angle_deg,gear_ratio); //计算减速比后的输出轴角度
	
	int16_t PIDoutput = PID_Double_CycleAngle(&Gimbal_Yaw_In,&Gimbal_Yaw_Ex,TargetAngle,DJI_MFeedback[ID - 1].speed_rpm,Angletest,1.00f);

	//int16_t PIDoutput = PID_FF_Calculate_CycleAngle(&Gimbal_Yaw_FF,Angletest,TargetAngle);

	ESC_Control_Raw_Single(&hcan1,ID,PIDoutput);
}

void Motor_DJI_IMUPitchContral(float TargetAngle,float IMUAngle,uint8_t ID,uint16_t gear_ratio)
{
	TargetAngle = MyMath_normalize_m180_to_p180(TargetAngle);	
	//Angletest = MyMath_cal_output_angle(DJI_MFeedback[ID - 1].angle_deg,gear_ratio); //计算减速比后的输出轴角度
	
	int16_t PIDoutput = PID_Double_CycleAngle(&Gimbal_Pitch_In,&Gimbal_Pitch_Ex,TargetAngle,DJI_MFeedback[ID - 1].speed_rpm,IMUAngle,1.00f);
	//int16_t PIDoutput = PID_FF_Calculate_CycleAngle(&Gimbal_Yaw_FF,Angletest,TargetAngle);

	ESC_Control_Raw_Single(&hcan1,ID,PIDoutput);
}

void Motor_DJI_IMUYawContral(float TargetAngle,float IMUAngle,uint8_t ID,uint16_t gear_ratio)
{
	TargetAngle = MyMath_normalize_m180_to_p180(TargetAngle);	
	//Angletest = MyMath_cal_output_angle(DJI_MFeedback[ID - 1].angle_deg,gear_ratio); //计算减速比后的输出轴角度
	
	int16_t PIDoutput = PID_Double_CycleAngle(&Gimbal_Yaw_In,&Gimbal_Yaw_Ex,TargetAngle,DJI_MFeedback[ID - 1].speed_rpm,IMUAngle,1.00f);
	//int16_t PIDoutput = PID_FF_Calculate_CycleAngle(&Gimbal_Yaw_FF,Angletest,TargetAngle);

	ESC_Control_Raw_Single(&hcan1,ID,PIDoutput);
}

void DJI_MOTOR_STOP_ALL(CAN_HandleTypeDef *hcan)
{
	float rawvalue[8];
	ESC_Control_Amps_All(hcan,&ESC_C620_20A,rawvalue);
}

