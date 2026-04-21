#include "HOTRC_HT10A.h"

//接受结构体
HOTRC_Ctl_t RC_Ctl;
SbusFrame_t RC_Raw;

// 全局变量定义
uint8_t  sbus_rx_buf[25];
uint16_t sbus_ch[16];
uint8_t  sbus_flags;
volatile uint8_t sbus_new_frame = 0; // 中断标志，主循环读取


//** #################################################### **//
//** ================= 本地静态函数声明 ================= **//
//** ################################################### **//

static void HORRC_HT10A_GET_Ctl(SbusFrame_t* RC_Raw);
static bool parse_sbus(const uint8_t* buffer, size_t len, SbusFrame_t* out);

//** ############################################### **//
//** ================= 对外函数声明 ================= **//
//** ############################################### **//

void Remote_ControlInit(void)
{
	__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE | UART_FLAG_IDLE);
	// 启动DMA接收25字节
	HAL_UART_Receive_DMA(&huart3, sbus_rx_buf, 25);

	// 开启空闲中断
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART3)
//    {
//			 // 1. DMA 完成时已自动停止，直接解析
//			 // 注意：长度必须传 25！
//			 if (parse_sbus(sbus_rx_buf, 25, &RC_Raw))
//			 {
//					// 2. 直接使用结构体内的 bool 标志位，避免硬编码掩码错误
//					if (!RC_Raw.frame_lost && !RC_Raw.failsafe)
//					{
//							sbus_new_frame = 1; // 通知主循环
//					}
//			 }
//			 // 3. 重启 DMA 准备接收下一帧
//			 HAL_UART_Receive_DMA(huart, sbus_rx_buf, 25);
//    }
//}

/* 缓冲区扩大到 50 字节，保证错位时也能覆盖完整帧 */
uint8_t sbus_dma_buf[50] __attribute__((aligned(4)));

void HOTRC_CallBack(UART_HandleTypeDef *huart)
{
		bool frame_found = false;
		
		// 滑动窗口寻帧头（最多搜到第 25 字节，保证 i+24 不越界）
		for (int i = 0; i <= 25; i++)
		{
				if (sbus_dma_buf[i] == 0x0F)
				{
						// 强校验帧尾
						if (sbus_dma_buf[i + 24] == 0x00)
						{
								// 解析找到的完整帧
								if (parse_sbus(&sbus_dma_buf[i], 25, &RC_Raw))
								{
										if (!RC_Raw.frame_lost && !RC_Raw.failsafe)
												sbus_new_frame = 1;
								}
								frame_found = true;
								break; // 每次只处理一帧，避免重复
						}
				}
		}

		// ?? 无论是否找到，立即重启 DMA 准备下一批 50 字节
		HAL_UART_Receive_DMA(huart, sbus_dma_buf, 50);
}

void HOTRC_ErrorCallback(UART_HandleTypeDef *huart)
{
	// 安全复位HAL状态机 + 停止当前DMA
	HAL_UART_AbortReceive(huart);
	
	// 立即重启接收，等待遥控器信号恢复
	HAL_UART_Receive_DMA(huart, sbus_dma_buf, 50);
}


//** ############################################### **//
//** ================= 本地静态函数 ================= **//
//** ############################################### **//
static uint8_t Switch_Set(uint16_t ChValue)
{
		if(ChValue > (192 - 20) && ChValue < (192 + 20))
		{
			return 1;
		}
		else if(ChValue > (992 - 20) && ChValue < (992 + 20))
		{
			return 2;
		}
		else if(ChValue > (1792 - 20) && ChValue < (1792 + 20))
		{
			return 3;
		}
}

static void HORRC_HT10A_GET_Ctl(SbusFrame_t* RC_Raw)
{
		RC_Ctl.Stick.LX = RC_Raw->channels[3] - HOTRC_MID_VEL;
		RC_Ctl.Stick.LY = RC_Raw->channels[2] - HOTRC_MID_VEL;
		RC_Ctl.Stick.RX = RC_Raw->channels[0] - HOTRC_MID_VEL;
		RC_Ctl.Stick.RY = RC_Raw->channels[1] - HOTRC_MID_VEL;
	
		RC_Ctl.Switch.S2_L = Switch_Set(RC_Raw->channels[5]);
		RC_Ctl.Switch.S2_R = Switch_Set(RC_Raw->channels[6]);
		RC_Ctl.Switch.S3_L = Switch_Set(RC_Raw->channels[4]);
		RC_Ctl.Switch.S3_R = Switch_Set(RC_Raw->channels[7]);
		
		RC_Ctl.Knob.KL = RC_Raw->channels[8];
	  RC_Ctl.Knob.KR = RC_Raw->channels[9];
	
		RC_Ctl.Flag.ch17 = RC_Raw->ch17;
		RC_Ctl.Flag.ch18 = RC_Raw->ch18;
		RC_Ctl.Flag.failsafe = RC_Raw->failsafe;
		RC_Ctl.Flag.frame_lost = RC_Raw->frame_lost;
		RC_Ctl.Flag.valid = RC_Raw->valid;
}

/**
 * @brief 解析一帧SBUS数据
 * @param buffer 接收到的原始字节数组，标准帧长25字节
 * @param len    数组实际长度
 * @param out    输出解析结果的结构体指针
 * @return       true: 解析成功, false: 校验失败或数据异常
 */
static bool parse_sbus(const uint8_t* buffer, size_t len, SbusFrame_t* out) {
    if (!buffer || !out || len < 25) {
        return false;
    }

    // 1. 帧头校验 (标准SBUS起始字节为 0x0F)
    if (buffer[0] != 0x0F) {
        out->valid = false;
        return false;
    }

    // 2. 解析16个11bit通道 (小端连续打包)
    // 使用 uint16_t 防止左移时发生8位溢出
    out->channels[0]  = ((uint16_t)buffer[1]       | ((uint16_t)buffer[2] << 8))                   & 0x07FF;
    out->channels[1]  = ((uint16_t)buffer[2] >> 3  | ((uint16_t)buffer[3] << 5))                   & 0x07FF;
    out->channels[2]  = ((uint16_t)buffer[3] >> 6  | ((uint16_t)buffer[4] << 2) | ((uint16_t)buffer[5] << 10)) & 0x07FF;
    out->channels[3]  = ((uint16_t)buffer[5] >> 1  | ((uint16_t)buffer[6] << 7))                   & 0x07FF;
    out->channels[4]  = ((uint16_t)buffer[6] >> 4  | ((uint16_t)buffer[7] << 4))                   & 0x07FF;
    out->channels[5]  = ((uint16_t)buffer[7] >> 7  | ((uint16_t)buffer[8] << 1) | ((uint16_t)buffer[9] << 9)) & 0x07FF;
    out->channels[6]  = ((uint16_t)buffer[9] >> 2  | ((uint16_t)buffer[10] << 6))                  & 0x07FF;
    out->channels[7]  = ((uint16_t)buffer[10] >> 5 | ((uint16_t)buffer[11] << 3))                  & 0x07FF;
    // 通道8~15的规律与0~7完全一致，只是字节偏移了11个(88bit)
    out->channels[8]  = ((uint16_t)buffer[12]      | ((uint16_t)buffer[13] << 8))                  & 0x07FF;
    out->channels[9]  = ((uint16_t)buffer[13] >> 3 | ((uint16_t)buffer[14] << 5))                  & 0x07FF;
    out->channels[10] = ((uint16_t)buffer[14] >> 6 | ((uint16_t)buffer[15] << 2) | ((uint16_t)buffer[16] << 10)) & 0x07FF;
    out->channels[11] = ((uint16_t)buffer[16] >> 1 | ((uint16_t)buffer[17] << 7))                  & 0x07FF;
    out->channels[12] = ((uint16_t)buffer[17] >> 4 | ((uint16_t)buffer[18] << 4))                  & 0x07FF;
    out->channels[13] = ((uint16_t)buffer[18] >> 7 | ((uint16_t)buffer[19] << 1) | ((uint16_t)buffer[20] << 9)) & 0x07FF;
    out->channels[14] = ((uint16_t)buffer[20] >> 2 | ((uint16_t)buffer[21] << 6))                  & 0x07FF;
    out->channels[15] = ((uint16_t)buffer[21] >> 5 | ((uint16_t)buffer[22] << 3))                  & 0x07FF;

    // 3. 解析标志位 (通常位于第24字节，索引为23或24，请以实际转接板手册为准)
    // 这里假设标志位在 buffer[23]
    uint8_t flags = buffer[23];
    out->ch17       = (flags & 0x80) ? true : false; // bit7
    out->ch18       = (flags & 0x40) ? true : false; // bit6
    out->frame_lost = (flags & 0x20) ? true : false; // bit5
    out->failsafe   = (flags & 0x10) ? true : false; // bit4
		
    out->valid = true;
		
		HORRC_HT10A_GET_Ctl(out);
		
    return true;
}





