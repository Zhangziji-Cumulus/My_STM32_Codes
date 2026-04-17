#ifndef __MOTOR_CONTRAL_H
#define __MOTOR_CONTRAL_H

#include "main.h"
#include "MY_PID.h"
#include "DJI_Motor_CAN.h"
#include "can.h"
#include "My_Math.h"

void Motor_CInit(void);
void Motor_DJI_Speed_SingleContral(int16_t MotorVel);
void Motor_DJI_Angle_SingleContral(float TargetAngle);
double MyMath_cal_output_angle(double CurentAngle,uint16_t gear_ratio);
#endif
