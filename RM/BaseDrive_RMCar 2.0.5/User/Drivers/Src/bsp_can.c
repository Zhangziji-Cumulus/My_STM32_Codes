#include "bsp_can.h"

extern motor_measure_t motor_chassis[7];
float DualBoard_ReceiveData[64] = {0.0f};

uint8_t  BOARD_ID_Int = BOARD_ID;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//motor data read
#define get_motor_measure(ptr, data)                                       \
	{                                                                      \
        (ptr)->last_ecd = (ptr)->ecd;                                      \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);               \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);         \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);     \
        (ptr)->temperate = (data)[6];                                      \
		(ptr)->Degree_Angle = EncoderToDegree((ptr)->ecd);                 \
		(ptr)->Radian_Angle = EncoderToRadian((ptr)->ecd);                 \
    }																	   

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	
}

void FloatToBytes(float value, uint8_t* bytes) {
    uint32_t* intValue = (uint32_t*)&value;
    bytes[0] = (*intValue >> 0) & 0xFF;
    bytes[1] = (*intValue >> 8) & 0xFF;
    bytes[2] = (*intValue >> 16) & 0xFF;
    bytes[3] = (*intValue >> 24) & 0xFF;
}

float BytesToFloat(uint8_t* bytes) {
    uint32_t intValue = (bytes[3] << 24) | (bytes[2] << 16) | (bytes[1] << 8) | bytes[0];
    return *((float*)&intValue);
}
 
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1)
	{
	    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_YAW_MOTOR_ID:
		case CAN_YAW2_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
		case CAN_PIT2_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_chassis[i], rx_data);
            break;
        }

        default:
        {
            break;
        }
    }
   }
  if(hcan->Instance == CAN2)
  {	 
	static uint8_t frameBuffer[64];  /* 存储2帧共16字节数据 */
    static uint32_t frameMask = 0;    /* 位掩码：记录已接收的帧 */
    static uint8_t Rx_TotalFrames;
	  
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    
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
                    DualBoard_ReceiveData[i-2] = BytesToFloat(&frameBuffer[i * 4]);
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

}
/* 带帧序号的CAN发送函数 */
bool CAN_SendFloatArray(CAN_HandleTypeDef* hcan, float* data, uint8_t length) { 
	
	if(length > 64 ){length = 64;}
	
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;
	
	if (length % 2 != 0) {
		length = length + 1; 
    } 
	
    uint8_t frames = (length) / 2;  /* 计算需要的帧数 */
    
	TxHeader.StdId = 0;       
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    for (uint8_t i = 0; i < frames + 1; i++) {
        memset(TxData, 0, 8);
        
		if(i == 0)
		{
			//TxData[0] = length;
			TxData[0] = frames;//总帧数
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
        TxHeader.ExtId = TX_BASE_ID + i;
        TxHeader.DLC = 8;
        
        if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
            Error_Handler();
        }
        
        /* 等待发送完成 */
        uint32_t timeout = 1000;
        while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3 && timeout-- > 0);
        if (timeout == 0) {
            Error_Handler();
        }
    }
}

