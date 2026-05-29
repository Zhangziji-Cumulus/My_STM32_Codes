#ifndef ZDT_CAN_BSP_H_
#define ZDT_CAN_BSP_H_

#include "can.h"
#include "stdbool.h"

//发送数据结构体
typedef struct {
	__IO CAN_RxHeaderTypeDef CAN_RxMsg;
	__IO uint8_t rxData[32];
	
	__IO CAN_TxHeaderTypeDef CAN_TxMsg;
	__IO uint8_t txData[32];

	__IO bool rxFrameFlag;
}CAN_t;
//接受数据结构体
typedef struct {
	int16_t current_ma;//电机实际运行电流
	struct
	{
		uint8_t Oac_TF;//掉电标志
		uint8_t Esi_RF;//右限位开关的状态
		uint8_t Esi_LF;//左限位开关的状态
		uint8_t Cgp_TF;//堵转保护标志
		uint8_t Cgi_TF;//堵转标志
		uint8_t Prf_TF;//位置到达标志
		uint8_t Ens_TF;//使能状态标志
	}flag;
}ZDT_FeedBack_t;

extern ZDT_FeedBack_t  ZDT_FeedBack;

void can_SendCmd(__IO uint8_t *cmd, uint8_t len);
void CAN_ZDT_Motor_FeedBack(ZDT_FeedBack_t* ZDT_FeedBack,uint8_t expectedId,CAN_RxHeaderTypeDef *pHeader, uint8_t* rxdata);

#endif // ZDT_CAN_BSP_H_
