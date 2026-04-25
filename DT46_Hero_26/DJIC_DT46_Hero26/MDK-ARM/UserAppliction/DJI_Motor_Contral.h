#ifndef __MOTOR_CONTRAL_H
#define __MOTOR_CONTRAL_H

#include "main.h"
#include "MY_PID.h"
#include "DJI_Motor_CAN.h"
#include "can.h"
#include "My_Math.h"

void Motor_Init(void);
void Motor_DJI_Speed_SingleContral(int16_t MotorVel);
void Motor_DJI_Angle_SingleContral(DJI_MotorFeedback_t* DJI_MFeedback,float TargetAngle,uint8_t ID,uint16_t gear_ratio);


void Motor_DJI_IMUPitchContral(DJI_MotorFeedback_t* DJI_MFeedback,float TargetAngle,float IMUAngle,uint8_t ID,uint16_t gear_ratio);
void Motor_DJI_IMUYawContral(DJI_MotorFeedback_t* DJI_MFeedback,float TargetAngle,float IMUAngle,uint8_t ID,uint16_t gear_ratio);

//¼±Í£
void DJI_MOTOR_STOP_ALL(CAN_HandleTypeDef *hcan);

void Motor_DJI_SpeedCtl_1_4(DJI_MotorFeedback_t* DJI_MFeedback,
														CAN_HandleTypeDef *hcan,
														float   MError,
														int16_t MotorVel_1,
														int16_t MotorVel_2,
														int16_t MotorVel_3,
														int16_t MotorVel_4);

void Motor_DJI_SpeedCtl_5_8(DJI_MotorFeedback_t* DJI_MFeedback,
														CAN_HandleTypeDef *hcan,
	                          float   MError,
														int16_t MotorVel_1,
														int16_t MotorVel_2,
														int16_t MotorVel_3,
														int16_t MotorVel_4);

void Motor_DJI_Speed_8Contral(DJI_MotorFeedback_t* DJI_MFeedback,
															CAN_HandleTypeDef *hcan,
															float   MError,
	                            int16_t MotorVel_1,
															int16_t MotorVel_2,
															int16_t MotorVel_3,
															int16_t MotorVel_4,
															int16_t MotorVel_5,
															int16_t MotorVel_6,
															int16_t MotorVel_7,
															int16_t MotorVel_8);
#endif
