#ifndef ZDT_CAN_BSP_H_
#define ZDT_CAN_BSP_H_

#include "can.h"
#include "stdbool.h"

typedef struct {
	__IO CAN_RxHeaderTypeDef CAN_RxMsg;
	__IO uint8_t rxData[32];
	
	__IO CAN_TxHeaderTypeDef CAN_TxMsg;
	__IO uint8_t txData[32];

	__IO bool rxFrameFlag;
}CAN_t;

void can_SendCmd(__IO uint8_t *cmd, uint8_t len);

#endif // ZDT_CAN_BSP_H_