#include "FDCAN_PART.h"
#include <string.h>

// 辅助函数：浮点数转字节（和原代码完全一致，直接复用）
static void FloatToBytes(float value, uint8_t* bytes) {
    uint32_t* intValue = (uint32_t*)&value;
    bytes[0] = (*intValue >> 0) & 0xFF;
    bytes[1] = (*intValue >> 8) & 0xFF;
    bytes[2] = (*intValue >> 16) & 0xFF;
    bytes[3] = (*intValue >> 24) & 0xFF;
}

// 辅助函数：字节转浮点数（和原代码完全一致，直接复用）
static float BytesToFloat(uint8_t* bytes) {
    uint32_t intValue = (bytes[3] << 24) | (bytes[2] << 16) | (bytes[1] << 8) | bytes[0];
    return *((float*)&intValue);
}

//** ############################################# **//
//** ================= FDCAN初始化 ================= **//
//** ############################################# **//

/**
 * @brief  FDCAN接收所有ID配置函数（标准帧+扩展帧全接收）
 * @param  hfdcan: FDCAN外设句柄（&hfdcan1 / &hfdcan2 / &hfdcan3）
 * @param  filter_index: 指定使用的过滤器索引
 * @retval HAL_StatusTypeDef: HAL_OK=成功
 */
HAL_StatusTypeDef FDCAN_Filter_AcceptAllID(FDCAN_HandleTypeDef *hfdcan, uint32_t filter_index)
{
    // 1. 参数校验
    if (hfdcan == NULL) return HAL_ERROR;

    // 2. 配置FDCAN过滤器（掩码模式+全0掩码=接收所有ID）
    FDCAN_FilterTypeDef fdcan_filter_st;
    fdcan_filter_st.IdType = FDCAN_STANDARD_ID;          // 标准ID
    fdcan_filter_st.FilterIndex = filter_index;           // 过滤器索引
    fdcan_filter_st.FilterType = FDCAN_FILTER_MASK;       // 掩码模式
    fdcan_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 放入FIFO0
    fdcan_filter_st.FilterID1 = 0x00000000;               // ID
    fdcan_filter_st.FilterID2 = 0x00000000;               // 掩码=全接收

    // 3. 应用过滤器
    if (HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter_st) != HAL_OK) {
        return HAL_ERROR;
    }

    // 4. 全局过滤器：所有未匹配帧也接收
    HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,  // 标准帧
                                 FDCAN_ACCEPT_IN_RX_FIFO0,  // 扩展帧
                                 DISABLE,                   // 远程帧
                                 DISABLE);                  // 远程帧

    // 5. 启动FDCAN
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        return HAL_ERROR;
    }

    // 6. 开启FIFO0新消息中断
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief  FDCAN1 / FDCAN2 / FDCAN3 三路过滤器初始化
 * @note   对应你原来的 can_filter_init
 */
void fdcan_filter_init(void)
{
    // 三路FDCAN分别使用不同的过滤器索引，避免冲突
    FDCAN_Filter_AcceptAllID(&hfdcan1, 0);  // FDCAN1 → 过滤器0
    FDCAN_Filter_AcceptAllID(&hfdcan2, 1);  // FDCAN2 → 过滤器1
    FDCAN_Filter_AcceptAllID(&hfdcan3, 2);  // FDCAN3 → 过滤器2
	
		HAL_Delay(100);
		// 直接读 FIFO 是否有消息（不依赖中断）
		if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0){
				// 这里如果能进来，说明硬件收到了，只是中断没开
				while(1); // 停在这里
		}
}

//** ############################################### **//
//** ================= FDCAN发送函数 ================= **//
//** ############################################### **//

/**
  * @brief  标准帧FDCAN发送函数（兼容传统CAN设备）
  * @param  hfdcan: FDCAN 句柄指针，id：ID，data：数据
  * @retval 无
  */
void FDCAN_Send_STD(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t* data) {
    FDCAN_TxHeaderTypeDef TxHeader;

    // 1. 配置发送帧头（兼容经典CAN模式）
    TxHeader.Identifier = id;                  // 标准帧 ID
    TxHeader.IdType = FDCAN_STANDARD_ID;       // 标准帧
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;   // 数据帧
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;   // 数据长度 8 字节
    TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;    // 关闭波特率切换（兼容老设备）
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;     // 经典CAN格式（非FD）
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // 2. 等待发送FIFO空闲
    while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0) {
        // 死等直到有空闲空间
    }
		
    // 3. 添加到发送FIFO并发送
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) != HAL_OK) {
        // 发送失败处理
        return;
    }
}

/**
 * @brief  带帧序号的FDCAN发送函数（使用扩展帧，对应原代码的 CAN_SendFloatArray）
 */
bool FDCAN_SendFloatArray(FDCAN_HandleTypeDef* hfdcan, float* data, uint8_t length, uint16_t ID) { 
	
	if(length > 64 ){length = 64;}
	
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
	
	if (length % 2 != 0) {
		length = length + 1; 
    } 
	
    uint8_t frames = (length) / 2;  /* 计算需要的帧数 */
    
    // 配置公共帧头参数
    TxHeader.Identifier = 0;       
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    
    for (uint8_t i = 0; i < frames + 1; i++) {
        memset(TxData, 0, 8);
        
		if(i == 0)
		{
			TxData[0] = frames; // 总帧数
		}
		else if(i > 0)
		{
			/* 填充浮点数数据 */
			if ((i-1) * 2 < length) {
				FloatToBytes(data[(i-1) * 2], &TxData[0]);
			}
			if ((i-1) * 2 + 1 < length) {
				FloatToBytes(data[(i-1) * 2 + 1], &TxData[4]);
			}
		}
		
        /* 使用ID的低5位作为帧序号*/
        TxHeader.Identifier = ID + i;
        TxHeader.DataLength = FDCAN_DLC_BYTES_8;
        
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) != HAL_OK) {
            Error_Handler();
        }
        
        /* 等待发送完成 */
        uint32_t timeout = 1000;
        while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) != 3 && timeout-- > 0);
        if (timeout == 0) {
            Error_Handler();
        }
    }
    return true;
}