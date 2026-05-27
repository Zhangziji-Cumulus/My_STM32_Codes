#include "ZDT_CAN_Bsp.h"

__IO CAN_t CAN1_Instance = {0};

/**
	* @brief   CAN发送多个字节
	* @param   无
	* @retval  无
	*/
void can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
	static uint32_t TxMailbox; __IO uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;

	// 除去ID地址和功能码后的数据长度
	j = len - 2;

	// 发送数据
	while(i < j)
	{
		// 数据个数
		k = j - i;

		// 填充缓存
		CAN1_Instance.CAN_TxMsg.StdId = 0x00;
		CAN1_Instance.CAN_TxMsg.ExtId = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;
		CAN1_Instance.txData[0] = cmd[1];
		CAN1_Instance.CAN_TxMsg.IDE = CAN_ID_EXT;
		CAN1_Instance.CAN_TxMsg.RTR = CAN_RTR_DATA;

		// 小于8字节命令
		if(k < 8)
		{
			for(l=0; l < k; l++,i++) { CAN1_Instance.txData[l + 1] = cmd[i + 2]; } CAN1_Instance.CAN_TxMsg.DLC = k + 1;
		}
		// 大于8字节命令，分包发送，每包数据最多发送8个字节
		else
		{
			for(l=0; l < 7; l++,i++) { CAN1_Instance.txData[l + 1] = cmd[i + 2]; } CAN1_Instance.CAN_TxMsg.DLC = 8;
		}

		// 发送数据
		while(HAL_CAN_AddTxMessage((&hcan1), (CAN_TxHeaderTypeDef *)(&CAN1_Instance.CAN_TxMsg), (uint8_t *)(&CAN1_Instance.txData), (&TxMailbox)) != HAL_OK);

		// 记录发送的第几包的数据
		++packNum;
	}
}
