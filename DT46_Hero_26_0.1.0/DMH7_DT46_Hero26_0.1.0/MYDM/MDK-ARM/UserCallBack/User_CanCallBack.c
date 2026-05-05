#include "User_CanCallBack.h"
#include "DJI_Motor_CAN.h"

//** ########################################### **//
//** ================= FDCAN总线 ================= **//
//** ########################################## **//

extern float DualBoard_ReceiveDataBuff[64];
extern DJI_MotorFeedback_t DJI_MFeedback_CAN1[8];
extern DJI_MotorFeedback_t DJI_MFeedback_CAN2[8];
extern DJI_MotorFeedback_t DJI_MFeedback_CAN3[8];

// 辅助函数（和原代码完全一致，直接复用）
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

/* 配置宏定义（和原代码完全一致） */
#define CAN2_MAX_DATA_FRAMES  8
#define CAN_FRAME_DATA_SIZE   8
#define CAN_RX_TIMEOUT_MS     1000

/* 接收状态结构体（和原代码完全一致） */
typedef struct {
    uint8_t  totalFrames;
    uint32_t receivedMask;
    uint8_t  buffer[CAN2_MAX_DATA_FRAMES * CAN_FRAME_DATA_SIZE + 8];
    uint32_t lastRxTick;
    bool     isReceiving;
} Can2RxState_t;

static Can2RxState_t g_can2RxState = {0};

/**
  * @brief  FDCAN 接收处理函数（对应原代码的 CAN_RxProcess）
  * @note   业务逻辑 100% 未动，只改了 HAL 库接口
  */
void FDCAN_RxProcess(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *pHeader, uint8_t RxData[])
{
    /* 2. 解析扩展ID（逻辑不变，只是用了 FDCAN 的结构体） */
    uint32_t baseId  = pHeader->Identifier & 0xFFFFFFE0;
    uint32_t frameId = pHeader->Identifier & 0x1F;

    if (baseId != RX_BASE_ID) {
        return;
    }

    /* 3. 超时保护（完全不变） */
    if (g_can2RxState.isReceiving && (HAL_GetTick() - g_can2RxState.lastRxTick > CAN_RX_TIMEOUT_MS)) {
        g_can2RxState.isReceiving = false;
        g_can2RxState.receivedMask = 0;
    }

    /* 4. 处理第0帧（完全不变） */
    if (frameId == 0) {
        g_can2RxState.totalFrames = RxData[0];
        
        if (g_can2RxState.totalFrames == 0 || g_can2RxState.totalFrames > CAN2_MAX_DATA_FRAMES) {
            g_can2RxState.isReceiving = false;
            return;
        }

        g_can2RxState.receivedMask = 0;
        g_can2RxState.isReceiving = true;
        g_can2RxState.lastRxTick = HAL_GetTick();
        return;
    }

    /* 5. 处理数据帧（完全不变） */
    if (!g_can2RxState.isReceiving || frameId > g_can2RxState.totalFrames) {
        return;
    }

    uint32_t frameBit = (1UL << (frameId - 1));
    if (g_can2RxState.receivedMask & frameBit) {
        return;
    }

    uint32_t bufOffset = frameId * CAN_FRAME_DATA_SIZE; 
    memcpy(&g_can2RxState.buffer[bufOffset], RxData, CAN_FRAME_DATA_SIZE);

    g_can2RxState.receivedMask |= frameBit;
    g_can2RxState.lastRxTick = HAL_GetTick();

    /* 6. 检查是否全部接收完成（完全不变） */
    uint32_t expectedMask = (1UL << g_can2RxState.totalFrames) - 1;
    if ((g_can2RxState.receivedMask & expectedMask) == expectedMask) {
        uint32_t floatCount = g_can2RxState.totalFrames * 2;
        for (uint32_t i = 0; i < floatCount; i++) {
            DualBoard_ReceiveDataBuff[i] = BytesToFloat(&g_can2RxState.buffer[(i + 2) * 4]);
        }

        g_can2RxState.isReceiving = false;
        g_can2RxState.receivedMask = 0;

        Dual_Board_ReceiveCallBack();
    }
}

/**
  * @brief  FDCAN FIFO0 接收回调函数（替换原 HAL_CAN_RxFifo0MsgPendingCallback）
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // 检查是否是“新消息到来”中断
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        FDCAN_RxHeaderTypeDef Temp_RxHeader;
        uint8_t Temp_RxData[8];

        // 【修改】循环读空 FIFO，函数名改为 HAL_FDCAN_GetRxMessage
        while(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &Temp_RxHeader, Temp_RxData) == HAL_OK)
        {
            // 判断是 FDCAN1 还是 FDCAN2
            if(hfdcan->Instance == FDCAN1)
            {
                // 【修改】判断帧类型：IdType 替代 IDE
                if (Temp_RxHeader.IdType == FDCAN_STANDARD_ID)
                {
                    // 标准帧 - 处理 DJI 电机数据
                    CAN_DJI_Motor_Feedback(DJI_MFeedback_CAN1, Temp_RxHeader.Identifier, Temp_RxData);
                }
                else if(Temp_RxHeader.IdType == FDCAN_EXTENDED_ID)
                {
                    // 扩展帧 - 双板通信
                    FDCAN_RxProcess(hfdcan, &Temp_RxHeader, Temp_RxData);
                } 	
            }
            else if (hfdcan->Instance == FDCAN2)
            {	
                // 【修改】判断帧类型
                if (Temp_RxHeader.IdType == FDCAN_STANDARD_ID)
                {
                    // 标准帧 - 电机
                    CAN_DJI_Motor_Feedback(DJI_MFeedback_CAN2, Temp_RxHeader.Identifier, Temp_RxData);
                }
                else if(Temp_RxHeader.IdType == FDCAN_EXTENDED_ID)
                {
                    // 扩展帧 - 双板通信
                    FDCAN_RxProcess(hfdcan, &Temp_RxHeader, Temp_RxData);
                }
            }
						else if(hfdcan->Instance == FDCAN3)
						{
						    // 【修改】判断帧类型
                if (Temp_RxHeader.IdType == FDCAN_STANDARD_ID)
                {
                    // 标准帧 - 电机
                    CAN_DJI_Motor_Feedback(DJI_MFeedback_CAN3, Temp_RxHeader.Identifier, Temp_RxData);
                }
                else if(Temp_RxHeader.IdType == FDCAN_EXTENDED_ID)
                {
                    // 扩展帧 - 双板通信
                    FDCAN_RxProcess(hfdcan, &Temp_RxHeader, Temp_RxData);
                }
						}
        }
    }
}