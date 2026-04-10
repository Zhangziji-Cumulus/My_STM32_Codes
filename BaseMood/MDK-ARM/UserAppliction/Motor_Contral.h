#ifndef __MOTOR_CONTRAL_H
#define __MOTOR_CONTRAL_H

#include "main.h"
#include "MY_PID.h"
#include "DJI_Motor_CAN.h"
#include "can.h"

void Motor_CInit(void);
void Motor_DJI_Speed_SingleContral(int16_t MotorVel);

#endif
