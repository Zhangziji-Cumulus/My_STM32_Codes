#include "DJI_Motor_Contral.h"

extern DJI_MotorFeedback_t DJI_MFeedback_CAN1[8];
extern DJI_MotorFeedback_t DJI_MFeedback_CAN2[8];
extern DJI_MotorFeedback_t DJI_MFeedback_CAN3[8];
extern FDCAN_HandleTypeDef hfdcan1; // 【修改】声明 FDCAN 句柄
extern FDCAN_HandleTypeDef hfdcan2; // 【修改】声明 FDCAN 句柄

PID_HandleTypeDef test2;
PID_HandleTypeDef test3;

PID_HandleTypeDef Motor_STOP;
PID_HandleTypeDef Motor_STOP_Ex;

PID_HandleTypeDef Chassis_MotorFL_In;
PID_HandleTypeDef Chassis_MotorFL_Ex;
PID_HandleTypeDef Chassis_MotorFR_In;
PID_HandleTypeDef Chassis_MotorFR_Ex;
PID_HandleTypeDef Chassis_MotorBL_In;
PID_HandleTypeDef Chassis_MotorBL_Ex;
PID_HandleTypeDef Chassis_MotorBR_In;
PID_HandleTypeDef Chassis_MotorBR_Ex;

PID_HandleTypeDef Chassis_Follow_PID;

PID_FF_HandleTypeDef Gimbal_Yaw_FF;
PID_HandleTypeDef Gimbal_Yaw_In;
PID_HandleTypeDef Gimbal_Yaw_Ex;

PID_FF_HandleTypeDef Gimbal_Pitch_FF;
PID_HandleTypeDef Gimbal_Pitch_In;
PID_HandleTypeDef Gimbal_Pitch_Ex;

PID_HandleTypeDef SFri_UL_In;
PID_HandleTypeDef SFri_UL_Ex;

PID_HandleTypeDef SFri_UR_In;
PID_HandleTypeDef SFri_UR_Ex;

PID_HandleTypeDef SFri_DM_In;
PID_HandleTypeDef SFri_DM_Ex;

PID_HandleTypeDef Dial_In;
PID_HandleTypeDef Dial_Ex;

// 【注意】Motor_Init 函数完全不需要修改，因为只初始化 PID，和硬件无关
void Motor_Init(void)
{
	uint8_t board_id = BOARD_ID; 
	
	//3508电机急停
  PID_Init(&Motor_STOP,3.0f,0.0f,0.0f,-DJI_M3508_R,DJI_M3508_R,-5.0f, 5.0f);
	
	//地盘PID
	PID_Init(&Chassis_MotorFL_In,1.0f,0.0f,0.0f,-DJI_M3508_R,DJI_M3508_R,-50.0f, 50.0f);
	PID_Init(&Chassis_MotorFL_Ex,15.0f,0.0f,0.0f,-10000,10000,-10.0f, 10.0f);

	PID_Init(&Chassis_MotorFR_In,1.0f,0.0f,0.0f,-DJI_M3508_R,DJI_M3508_R,-50.0f, 50.0f);
	PID_Init(&Chassis_MotorFR_Ex,15.0f,0.0f,0.0f,-10000,10000,-10.0f, 10.0f);
	
	PID_Init(&Chassis_MotorBL_In,1.0f,0.0f,0.0f,-DJI_M3508_R,DJI_M3508_R,-50.0f, 50.0f);
	PID_Init(&Chassis_MotorBL_Ex,15.0f,0.0f,0.0f,-10000,10000,-10.0f, 10.0f);
	
	PID_Init(&Chassis_MotorBR_In,1.0f,0.0f,0.0f,-DJI_M3508_R,DJI_M3508_R,-50.0f, 50.0f);
	PID_Init(&Chassis_MotorBR_Ex,15.0f,0.0f,0.0f,-10000,10000,-10.0f, 10.0f);
	
	PID_Init(&Chassis_Follow_PID,1.7f,0.05f,0.0f,-10,10,-1.0f, 1.0f);
	
	//云台PID
	PID_FF_Init(&Gimbal_Yaw_FF,750.0f,0.05f,40.0f,0.0,-DJI_GM6020_R,DJI_GM6020_R,-10.0f, 10.0f);
	PID_Init(&Gimbal_Yaw_In,5.0f,0.0f,0.0f,-DJI_GM6020_R,DJI_GM6020_R,-10.0f, 10.0f);
	PID_Init(&Gimbal_Yaw_Ex,200.0f,0.0f,0.0f,-1000,1000,-10.0f, 10.0f);
	
	PID_FF_Init(&Gimbal_Pitch_FF,3.0f,0.0f,0.0f,1.0,-DJI_M3508_R,DJI_M3508_R,-10.0f, 10.0f);
	PID_Init(&Gimbal_Pitch_In,1.0f,0.0f,0.0f,-2000,2000,-10.0f, 10.0f);
	PID_Init(&Gimbal_Pitch_Ex,300.0f,0.0f,0.0f,-1500,1500,-10.0f, 10.0f);
	
	//摩擦轮PID
	PID_Init(&SFri_UL_In,1.0f,0.0f,0.0f,-DJI_M3508_R,DJI_M3508_R,-10.0f, 10.0f);
	PID_Init(&SFri_UL_Ex,30.0f,0.0f,0.0f,-10000,10000,-10.0f, 10.0f);
	
	PID_Init(&SFri_UR_In,1.0f,0.0f,0.0f,-DJI_M3508_R,DJI_M3508_R,-10.0f, 10.0f);
	PID_Init(&SFri_UR_Ex,30.0f,0.0f,0.0f,-10000,10000,-10.0f, 10.0f);
	
	PID_Init(&SFri_DM_In,1.0f,0.0f,0.0f,-DJI_M3508_R,DJI_M3508_R,-10.0f, 10.0f);
	PID_Init(&SFri_DM_Ex,30.0f,0.0f,0.0f,-10000,10000,-10.0f, 10.0f);
	
	PID_Init(&Dial_In,1.0f,0.0f,0.0f,-1500,1500,-10.0f, 10.0f);
	PID_Init(&Dial_Ex,30.0f,0.0f,0.0f,-10000,10000,-10.0f, 10.0f);
	
}

// 【修改】底盘控制函数
void Motor_DJI_Chassis(DJI_MotorFeedback_t DJI_MFeedback[],
														FDCAN_HandleTypeDef *hfdcan, // 【修改】参数类型
	                          float   MError,
														int16_t MotorVel_1,
														int16_t MotorVel_2,
														int16_t MotorVel_3,
														int16_t MotorVel_4){
					
	int16_t PIDoutput[4];	
																
	PIDoutput[CHASSIS_FL] = PID_Double_Calculate(&Chassis_MotorFL_In,
															 &Chassis_MotorFL_Ex, 
															 MotorVel_1,
															 DJI_MFeedback[CHASSIS_FL].current_ma,
															 DJI_MFeedback[CHASSIS_FL].speed_rpm,
															 MError);
																
	PIDoutput[CHASSIS_FR] = PID_Double_Calculate(&Chassis_MotorFR_In,
															 &Chassis_MotorFR_Ex, 
															 MotorVel_2,
															 DJI_MFeedback[CHASSIS_FR].current_ma,
															 DJI_MFeedback[CHASSIS_FR].speed_rpm,
															 MError);
																
  PIDoutput[CHASSIS_BL] = PID_Double_Calculate(&Chassis_MotorBL_In,
															 &Chassis_MotorBL_Ex, 
															 MotorVel_3,
															 DJI_MFeedback[CHASSIS_BL].current_ma,
															 DJI_MFeedback[CHASSIS_BL].speed_rpm,
															 MError);
															 
	PIDoutput[CHASSIS_BR] = PID_Double_Calculate(&Chassis_MotorBR_In,
															 &Chassis_MotorBR_Ex, 
															 MotorVel_4,
															 DJI_MFeedback[CHASSIS_BR].current_ma,
															 DJI_MFeedback[CHASSIS_BR].speed_rpm,
															 MError);
															 
	// 【修改】调用 FDCAN 版本的控制函数
	ESC_Control_Raw_Group(hfdcan, 1, PIDoutput);
}

// 【修改】摩擦轮控制函数
void Motor_DJI_ShootingFri(DJI_MotorFeedback_t DJI_MFeedback[],
														FDCAN_HandleTypeDef *hfdcan, // 【修改】参数类型
	                          float   MError,
														int16_t SFri_UL_M,
														int16_t SFri_UR_M,
														int16_t SFri_MD_M){
					
	int16_t PIDoutput[4];	
																
	PIDoutput[0] = PID_Double_Calculate(&SFri_UL_In,
																		  &SFri_UL_Ex, 
																		  SFri_UL_M,
																		  DJI_MFeedback[0].current_ma,
																		  DJI_MFeedback[0].speed_rpm,
																		  MError);
																
	PIDoutput[1] = PID_Double_Calculate(&SFri_UR_In,
																		  &SFri_UR_Ex, 
																			SFri_UR_M,
																		  DJI_MFeedback[1].current_ma,
																		  DJI_MFeedback[1].speed_rpm,
																		  MError);
																
  PIDoutput[2] = PID_Double_Calculate(&SFri_DM_In,
																		  &SFri_DM_Ex, 
																		  SFri_MD_M,
																		  DJI_MFeedback[2].current_ma,
																		  DJI_MFeedback[2].speed_rpm,
																		  MError);

  PIDoutput[3] = 0;

	// 【修改】调用 FDCAN 版本的控制函数
	ESC_Control_Raw_Group(hfdcan, 1, PIDoutput);
}
														
// 【修改】摩擦轮控制函数
void Motor_DJI_Dial(DJI_MotorFeedback_t DJI_MFeedback[],
														FDCAN_HandleTypeDef *hfdcan, // 【修改】参数类型
	                          float   MError,
														int16_t DialV){
					
	int16_t PIDoutput[4];	
																
	PIDoutput[0] = PID_Double_Calculate(&Dial_In,
																		  &Dial_Ex, 
																		  DialV,
																		  DJI_MFeedback[0].current_ma,
																		  DJI_MFeedback[0].speed_rpm,
																		  MError);

	// 【修改】调用 FDCAN 版本的控制函数
	ESC_Control_Raw_Group(hfdcan,1, PIDoutput);
}

// 【修改】单电机角度控制
void Motor_DJI_Angle_SingleContral(DJI_MotorFeedback_t DJI_MFeedback[], float TargetAngle, uint8_t ID, uint16_t gear_ratio)
{
	float Angletest = 0.0;
	TargetAngle = MyMath_normalize_m180_to_p180(TargetAngle);	
	Angletest = MyMath_cal_output_angle(DJI_MFeedback[ID - 1].angle_deg, gear_ratio);
	
	int16_t PIDoutput = PID_Double_CycleAngle(&Gimbal_Yaw_In, &Gimbal_Yaw_Ex, TargetAngle, DJI_MFeedback[ID - 1].speed_rpm, Angletest, 1.00f);

	// 【修改】句柄改为 hfdcan1
	ESC_Control_Raw_Single(&hfdcan1, ID, PIDoutput);
}

// 【修改】云台 Pitch 控制
void Motor_DJI_IMUPitchContral(DJI_MotorFeedback_t DJI_MFeedback[], float TargetAngle, float IMUAngle, uint8_t ID, uint16_t gear_ratio)
{
	int16_t PIDoutput = (PID_Double_Calculate(&Gimbal_Pitch_In, &Gimbal_Pitch_Ex, TargetAngle, DJI_MFeedback[ID - 1].speed_rpm, IMUAngle, 0.1f));
	
	// 【修改】句柄改为 hfdcan2
	ESC_Control_Raw_Single(&hfdcan2, ID, PIDoutput);
}

// 【修改】云台 Yaw 控制
void Motor_DJI_IMUYawContral(DJI_MotorFeedback_t DJI_MFeedback[], float TargetAngle, float IMUAngle, uint8_t ID, uint16_t gear_ratio)
{
	TargetAngle = MyMath_normalize_m180_to_p180(TargetAngle);	
	
	int16_t PIDoutput = PID_FF_Calculate_CycleAngle(&Gimbal_Yaw_FF, IMUAngle, TargetAngle);

	// 【修改】句柄改为 hfdcan2
	ESC_Control_Raw_Single(&hfdcan2, ID, PIDoutput);
}

// 【修改】急停函数
void DJI_MOTOR_EmergencySTOP_ALL(DJI_MotorFeedback_t DJI_MFeedback[], FDCAN_HandleTypeDef *hfdcan, float MError)
{
	int16_t PIDoutput1[4];	
	int16_t PIDoutput2[4];
	
	PIDoutput1[0] = PID_Calculate(&Motor_STOP, DJI_MFeedback[0].speed_rpm, 0);																
	PIDoutput1[1] = PID_Calculate(&Motor_STOP, DJI_MFeedback[1].speed_rpm, 0);
	PIDoutput1[2] = PID_Calculate(&Motor_STOP, DJI_MFeedback[2].speed_rpm, 0);
	PIDoutput1[3] = PID_Calculate(&Motor_STOP, DJI_MFeedback[3].speed_rpm, 0);
	
	PIDoutput2[0] = PID_Calculate(&Motor_STOP, DJI_MFeedback[4].speed_rpm, 0);
	PIDoutput2[1] = PID_Calculate(&Motor_STOP, DJI_MFeedback[5].speed_rpm, 0);
	PIDoutput2[2] = PID_Calculate(&Motor_STOP, DJI_MFeedback[6].speed_rpm, 0);
	PIDoutput2[3] = PID_Calculate(&Motor_STOP, DJI_MFeedback[7].speed_rpm, 0);
	
	// 【修改】调用 FDCAN 版本
	ESC_Control_Raw_Group(hfdcan, 1, PIDoutput1);
	ESC_Control_Raw_Group(hfdcan, 5, PIDoutput2); // 【注意】你原代码这里写的是4，应该是5，我帮你修正了
}