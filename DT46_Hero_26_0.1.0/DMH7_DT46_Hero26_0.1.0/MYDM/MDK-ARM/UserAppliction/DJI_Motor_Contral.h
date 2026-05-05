#ifndef __MOTOR_CONTRAL_H
#define __MOTOR_CONTRAL_H

#include "main.h"
#include "MY_PID.h"
#include "DJI_Motor_CAN.h" // 【修改】替换成 FDCAN 版本
#include "fdcan.h"
#include "My_Math.h"
#include "Dual_board_Transmit.h"

//定义轮子ID数组序号
#define CHASSIS_FL 3
#define CHASSIS_FR 2
#define CHASSIS_BL 0
#define CHASSIS_BR 1

void Motor_Init(void);
void Motor_DJI_Speed_SingleContral(int16_t MotorVel);
void Motor_DJI_Angle_SingleContral(DJI_MotorFeedback_t DJI_MFeedback[], float TargetAngle, uint8_t ID, uint16_t gear_ratio);

// 【修改】所有函数参数 CAN_HandleTypeDef 改为 FDCAN_HandleTypeDef
void Motor_DJI_Chassis(DJI_MotorFeedback_t DJI_MFeedback[],
											 FDCAN_HandleTypeDef *hfdcan,
											 float   MError,
											 int16_t MotorVel_1,
											 int16_t MotorVel_2,
										   int16_t MotorVel_3,
											 int16_t MotorVel_4);

void Motor_DJI_IMUPitchContral(DJI_MotorFeedback_t DJI_MFeedback[], float TargetAngle, float IMUAngle, uint8_t ID, uint16_t gear_ratio);
void Motor_DJI_IMUYawContral(DJI_MotorFeedback_t DJI_MFeedback[], float TargetAngle, float IMUAngle, uint8_t ID, uint16_t gear_ratio);

void Motor_DJI_ShootingFri(DJI_MotorFeedback_t DJI_MFeedback[],
														FDCAN_HandleTypeDef *hfdcan, // 【修改】
	                          float   MError,
														int16_t SFri_UL_M,
														int16_t SFri_UR_M,
														int16_t SFri_MD_M);

void Motor_DJI_Dial(DJI_MotorFeedback_t DJI_MFeedback[],
														FDCAN_HandleTypeDef *hfdcan, // 【修改】参数类型
	                          float   MError,
														int16_t DialV);

// 急停
void DJI_MOTOR_EmergencySTOP_ALL(DJI_MotorFeedback_t DJI_MFeedback[], FDCAN_HandleTypeDef *hfdcan, float MError); // 【修改】
void DJI_MOTOR_STOP_ALL(FDCAN_HandleTypeDef *hfdcan); // 【修改】

#endif