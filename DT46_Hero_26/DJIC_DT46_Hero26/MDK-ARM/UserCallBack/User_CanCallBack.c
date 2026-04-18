#include "User_CanCallBack.h"

float DualBoard_ReceiveDataBuff[64];

static void FloatToBytes(float value, uint8_t* bytes) {
    uint32_t* intValue = (uint32_t*)&value;
    bytes[0] = (*intValue >> 0) & 0xFF;
    bytes[1] = (*intValue >> 8) & 0xFF;
    bytes[2] = (*intValue >> 16) & 0xFF;
    bytes[3] = (*intValue >> 24) & 0xFF;
}

static float BytesToFloat(uint8_t* bytes) {
    uint32_t intValue = (bytes[3] << 24) | (bytes[2] << 16) | (bytes[1] << 8) | bytes[0];
    return *((float*)&intValue);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader;
	static uint8_t RxData[8];
	//HAL_StatusTypeDef status;	
//	
//	status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	
//	if(status == HAL_OK)
//	{
		if(hcan->Instance == CAN1)
		{
			 CAN_DJI_Motor_Feedback(RxHeader.StdId,RxData);    
		}
		else if(hcan->Instance == CAN2)
		{
			static uint8_t frameBuffer[64];  /* 存储2帧共16字节数据 */
			static uint32_t frameMask = 0;    /* 位掩码：记录已接收的帧 */
			static uint8_t Rx_TotalFrames;
						
			//CAN_RxHeaderTypeDef RxHeader;
			//uint8_t RxData[8];
						
			if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
								Error_Handler();
			}
						
			/* 提取基ID和帧序号 */
			uint32_t baseId = RxHeader.ExtId & 0xFFFFFFE0;  /* 清除低5位的帧序号 */
			uint32_t frameId = RxHeader.ExtId & 0x1F;   /* 提取低5位的帧序号 */
			
			/* 检查基ID是否匹配 */
			if (baseId == RX_BASE_ID) {
				if(frameId == 0)
				{
					Rx_TotalFrames = RxData[0];
				}
				else if(frameId > 0)
				{
					/* 检查帧序号范围 */
					if (frameId <= Rx_TotalFrames) {  /* 我们只发送64帧 */
							/* 复制数据到缓冲区 */
							memcpy(&frameBuffer[frameId * 8], RxData, 8);
							
							/* 更新帧接收掩码 */
							frameMask++; 
											
							/* 检查是否所有帧都已接收 */
							if (frameMask == Rx_TotalFrames) {  
									/* 所有帧接收完成，解析数据 */
									for (int i = 2; i < (Rx_TotalFrames * 2) + 2; i++) {
											DualBoard_ReceiveDataBuff[i-2] = BytesToFloat(&frameBuffer[i * 4]);
									}
													
									/* 重置状态 */
									frameMask = 0;
											
									/* 数据接收完成回调 */
									Dual_Board_ReceiveCallBack();
							 }
						}			
					}
			  }    
				HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);	  
			}
	//}
}
	

