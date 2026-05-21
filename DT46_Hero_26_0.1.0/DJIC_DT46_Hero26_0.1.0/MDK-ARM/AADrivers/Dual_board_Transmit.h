#ifndef __DUAL_BOARD_TRANSMIT_H__
#define __DUAL_BOARD_TRANSMIT_H__

#include "main.h"
#include "APP_Config.h"

#if(BOARD_MODE == BOARD_MODE_DUAL)

//双板通信任务系统循环时间
#define DUAL_BOARD_TRANSMIT_TASK_TIME_MS 1

#define GIMBAL_BOAD		1       //云台板
#define CHASSIS_BOAD	2       //地盘板

#define BOARD_ID        CHASSIS_BOAD       //云台板、地盘板

#define TX_BASE_ID ((BOARD_ID == 1) ? 0x100 : 0x200)  /* 发送基ID */
#define RX_BASE_ID ((BOARD_ID == 1) ? 0x200 : 0x100)  /* 接收基ID */

#endif

#endif /* __DUAL_BOARD_TRANSMIT_H__ */
