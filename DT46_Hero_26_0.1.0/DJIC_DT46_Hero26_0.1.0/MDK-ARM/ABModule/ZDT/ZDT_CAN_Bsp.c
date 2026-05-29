#include "ZDT_CAN_Bsp.h"

//** #################################################################################################### **//
//** ================================= ZDT CAN 发送函数(通过CAN2进行发送) ================================= **//
//** #################################################################################################### **//

//

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
		while(HAL_CAN_AddTxMessage((&hcan2), (CAN_TxHeaderTypeDef *)(&CAN1_Instance.CAN_TxMsg), (uint8_t *)(&CAN1_Instance.txData), (&TxMailbox)) != HAL_OK);

		// 记录发送的第几包的数据
		++packNum;
	}
}

//** #################################################################################################### **//
//** ========================================== 接受数据解析函数 ========================================= **//
//** #################################################################################################### **//

ZDT_FeedBack_t  ZDT_FeedBack;

void CAN_ZDT_Motor_FeedBack(ZDT_FeedBack_t* ZDT_FeedBack,uint8_t expectedId,CAN_RxHeaderTypeDef *pHeader, uint8_t* rxdata)
{
	uint32_t ext_id = pHeader->ExtId;		//获取CAN ID帧
	uint8_t Addr = (ext_id >> 8) & 0xFF;	// 提取设备地址
	uint8_t Packet = ext_id & 0xFF;      	// 提取包标识  

	
	uint8_t Code = rxdata[0];

	if(Addr == expectedId && Packet == 0)
	{
		switch(Code)
		{
			case 0x27: 
					uint8_t checkcode = rxdata[3];	
					if(checkcode == 0x6B)
					{
						ZDT_FeedBack->current_ma = rxdata[1] | rxdata[2]; 
					 	break;
					}
		}
	}

}

// CAN_DJI_Motor_Feedback(DJI_MFeedback_CAN1, Temp_RxHeader.StdId, Temp_RxData);

// /**
//  * @brief 解析 DJI 电机 CAN 反馈帧
//  * @param  DJI_MFeedback: 电机状态数组指针
//  * @param  std_id:        CAN 标准帧 ID (有效范围 0x201 ~ 0x208)
//  * @param  data:          8 字节 CAN 数据载荷
//  * @retval None
//  */
// void CAN_DJI_Motor_Feedback(DJI_MotorFeedback_t* DJI_MFeedback, uint32_t std_id, uint8_t* data)
// {
//     // 1. 校验 ID 是否在有效范围内 (协议规定: 0x200 + 电机ID)
//     // 仅支持 ID 1 ~ 8，对应 CAN ID 0x201 ~ 0x208
//     if (std_id >= 0x201 && std_id <= 0x208) 
//     {
//         uint8_t index = std_id - 0x201; // 将 CAN ID 映射为数组索引 0~7
        
//         DJI_MFeedback[index].id = index + 1;

//         // 收到数据 → 立刻重置超时计数器
//         DJI_MFeedback[index].offline_timeout_cnt = 0;

//         // 成功接收数据，标记该电机在线
//         DJI_MFeedback[index].is_online = 1; 

//         // 2. 解析机械角度 (DATA[0] 高8位, DATA[1] 低8位)
//         // 原始值范围 0 ~ 8191，对应物理角度 0° ~ 360°
//         DJI_MFeedback[index].angle_raw = (uint16_t)((data[0] << 8) | data[1]);
        
//         // 转换为实际角度值 (浮点型，按需使用)
//         DJI_MFeedback[index].angle_deg = (float)DJI_MFeedback[index].angle_raw * 360.0f / 8192.0f;

//         // 3. 解析转速 (DATA[2] 高8位, DATA[3] 低8位)
//         // 单位: rpm (转/分钟)
//         DJI_MFeedback[index].speed_rpm = (int16_t)((data[2] << 8) | data[3]);

//         // 4. 解析实际电流 (DATA[4] 高8位, DATA[5] 低8位)
//         // 注: 官方协议中该值为电调实际输出电流，单位通常为 mA
//         DJI_MFeedback[index].current_ma = (int16_t)((data[4] << 8) | data[5]);

//         // 5. 解析错误码 (DATA[7])
//         DJI_MFeedback[index].error_code = (DJI_MotorErrorCode_t)data[7];
        
//         // DATA[6] 为保留字段 (部分电调版本用于表示温度)，此处按协议忽略
//     }
// }