#include "User_CallBack.h"
#include "Dual_board_Transmit.h"
#include "bsp_CAN.h"

//** ####################################### **//
//** ================= 串口 ================= **//
//** ####################################### **//


//** ########################################### **//
//** ================= CAN总线 ================= **//
//** ########################################## **//

// 全局变量存储接收到的数据
static Example_Struct_t g_receivedData = {0};
static uint8_t g_dataValid = 0;

static CMD_t RxCMD = {0};

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
#if(BOARD_MODE == BOARD_MODE_DUAL)
					// 【接收解析】自动填充结构体
					if (DualBoard_ParseStruct(&Temp_RxHeader, Temp_RxData, 
					                         &RxCMD, sizeof(RxCMD))) {
						// 成功接收到完整结构体
						g_dataValid = 1;
						
						// 可以在这里设置标志位或发送到队列
						// 具体使用由用户在任务中决定
					}
#endif
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
			if (Temp_RxHeader.IDE == CAN_ID_STD)
			{
					// 标准帧 - 电机
					CAN_DJI_Motor_Feedback(DJI_MFeedback_CAN2, Temp_RxHeader.StdId, Temp_RxData);
			}
			else if(Temp_RxHeader.IDE == CAN_ID_EXT)
			{
#if(BOARD_MODE == BOARD_MODE_DUAL)
					// 【调试】打印扩展帧ID（如果有串口）
					// printf("RX ExtID: 0x%08X\n", Temp_RxHeader.ExtId);
					
					// 【接收解析】自动填充结构体
					if (DualBoard_ParseStruct(&Temp_RxHeader, Temp_RxData,
					                         &RxCMD, sizeof(RxCMD))) {
						g_dataValid = 1;
					}
#endif
			}
		}
	}
}
	
