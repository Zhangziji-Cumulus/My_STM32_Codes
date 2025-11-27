#ifndef __DUAL_BOARD_TRANSMIT_H
#define __DUAL_BOARD_TRANSMIT_H

#include "stm32f4xx.h"                  // Device header
#include "main.h"
#include "Remote_Control.h"
#include "bsp_can.h"
#include "struct_typedef.h"
#include "INS_task.h"
#include "Gimbal_Move.h"


void Dual_Board_Send(void);
void Dual_Board_ReceiveCallBack(void);

#endif
