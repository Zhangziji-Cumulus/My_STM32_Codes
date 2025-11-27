#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"
#include "can.h"
#include "stm32f4xx.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "Motors.h"
#include "Dual_board_Transmit.h"

//dual-board communication
#define BOARD_ID 2
#define TX_BASE_ID ((BOARD_ID == 1) ? 0x100 : 0x200)  /* 发送基ID */
#define RX_BASE_ID ((BOARD_ID == 1) ? 0x200 : 0x100)  /* 接收基ID */

//Using CAN2 for dual-board communication
void can_filter_init(void);
bool CAN_SendFloatArray(CAN_HandleTypeDef* hcan, float* data, uint8_t length);

#endif

