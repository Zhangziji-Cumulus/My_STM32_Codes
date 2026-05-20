#include "User_CallBack.h"

//** ####################################### **//
//** ================= 串口 ================= **//
//** ####################################### **//


//** ########################################### **//
//** ================= CAN总线 ================= **//
//** ########################################## **//



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	if(hcan->Instance == CAN1)
	{
		CAN_RxHeaderTypeDef Temp_RxHeader;
		uint8_t Temp_RxData[8];
			
		while(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Temp_RxHeader, Temp_RxData) == HAL_OK)
		{
			if (Temp_RxHeader.IDE == CAN_ID_STD)// 标准帧
			{
					// 处理 DJI 电机数据
					CAN_DJI_Motor_Feedback(DJI_MFeedback_CAN1,Temp_RxHeader.StdId, Temp_RxData);
			}
			else if(Temp_RxHeader.IDE == CAN_ID_EXT)// 扩展帧
			{
					//CAN_RxProcess(hcan,&Temp_RxHeader, Temp_RxData);
			} 	
		}
	}
    else if (hcan->Instance == CAN2)
    {	
		CAN_RxHeaderTypeDef Temp_RxHeader;
		uint8_t Temp_RxData[8];

		// 循环读空FIFO
		while(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Temp_RxHeader, Temp_RxData) == HAL_OK)
		{
			if (Temp_RxHeader.IDE == 0)
			{
					// 标准帧 - 电机
					CAN_DJI_Motor_Feedback(DJI_MFeedback_CAN2, Temp_RxHeader.StdId, Temp_RxData);
			}
			else if(Temp_RxHeader.IDE == 4)
			{
					// 扩展帧 - 双板通信 【现在绝对正确！】
					//CAN_RxProcess(hcan, &Temp_RxHeader, Temp_RxData);
			}
		}
	}
}
	
