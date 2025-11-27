#ifndef __RESET_TOINIT_H
#define __RESET_TOINIT_H

#include "stm32f4xx.h"                  // Device header
#include "Remote_Control.h"
#include "Chassis_Move.h"
#include "Gimbal_Move.h"
#include "Shooting.h"
#include "CAN_Bsp.h"

void GetReSetFlag(void);
void ReSet_ToInit(void);
void ReSet_LEDTip(void);
void UserInit(void);

#endif
