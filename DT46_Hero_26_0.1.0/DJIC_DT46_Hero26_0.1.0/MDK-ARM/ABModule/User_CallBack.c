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
static uint8_t g_dataValid = 0;

CMD_t RXCMD  = {0};

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef Temp_RxHeader;
    uint8_t Temp_RxData[8];

#if defined(CAN1)
    if (hcan->Instance == CAN1)
    {
        while (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Temp_RxHeader, Temp_RxData) == HAL_OK)
        {
            if (Temp_RxHeader.IDE == CAN_ID_STD)
            {
                CAN_DJI_Motor_Feedback(DJI_MFeedback_CAN1, Temp_RxHeader.StdId, Temp_RxData);
            }
#if (BOARD_MODE == BOARD_MODE_DUAL)
            else if (Temp_RxHeader.IDE == CAN_ID_EXT)
            {
                if (DualBoard_ParseStruct(&Temp_RxHeader, Temp_RxData, RX_BASE_ID, &RXCMD, sizeof(RXCMD)))
                {
                    g_dataValid = 1;
                }
            }
#endif
        }
    }
#endif

#if defined(CAN2)
    if (hcan->Instance == CAN2)
    {
        while (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Temp_RxHeader, Temp_RxData) == HAL_OK)
        {
            if (Temp_RxHeader.IDE == CAN_ID_STD)
            {
                CAN_DJI_Motor_Feedback(DJI_MFeedback_CAN2, Temp_RxHeader.StdId, Temp_RxData);
            }
#if (BOARD_MODE == BOARD_MODE_DUAL)
            else if (Temp_RxHeader.IDE == CAN_ID_EXT)
            {
                if (DualBoard_ParseStruct(&Temp_RxHeader, Temp_RxData, RX_BASE_ID, &RXCMD, sizeof(RXCMD)))
                {
                    g_dataValid = 1;
                }
            }
#endif
        }
    }
#endif

#if defined(CAN3)
    if (hcan->Instance == CAN3)
    {
        while (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Temp_RxHeader, Temp_RxData) == HAL_OK)
        {
            if (Temp_RxHeader.IDE == CAN_ID_STD)
            {
                CAN_DJI_Motor_Feedback(DJI_MFeedback_CAN3, Temp_RxHeader.StdId, Temp_RxData);
            }
#if (BOARD_MODE == BOARD_MODE_DUAL)
            else if (Temp_RxHeader.IDE == CAN_ID_EXT)
            {
                if (DualBoard_ParseStruct(&Temp_RxHeader, Temp_RxData, RX_BASE_ID, &RXCMD, sizeof(RXCMD)))
                {
                    g_dataValid = 1;
                }
            }
#endif
        }
    }
#endif
}
	
