#ifndef __DUAL_BOARD_TRANSMIT_H
#define __DUAL_BOARD_TRANSMIT_H

#include "main.h"
#include <stdint.h>

#define BOARD_ID 1

//#include "Remote_Control.h"
//#include "bsp_can.h"
//#include "INS_task.h"
//#include "Gimbal_Move.h"


void Dual_Board_Send(void);
void Dual_Board_ReceiveCallBack(void);

#endif
