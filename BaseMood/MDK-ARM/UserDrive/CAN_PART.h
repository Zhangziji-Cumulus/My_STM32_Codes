#ifndef __CAN_PART_H
#define __CAN_PART_H

#include "main.h"
#include "DJI_Motor.h"

void MY_CAN_Init(CAN_HandleTypeDef *hcan);
void CAN_Send_STD(CAN_HandleTypeDef *hcan,uint32_t id, uint8_t* data);

#endif
