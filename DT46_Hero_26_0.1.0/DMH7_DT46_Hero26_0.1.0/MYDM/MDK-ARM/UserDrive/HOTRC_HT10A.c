#include "HOTRC_HT10A.h"
#include "string.h"

//接受结构体
HOTRC_Ctl_t RC_Ctl;

//// 全局变量
//UART_HandleTypeDef *sbusHuart;
//uint8_t sbusRxBuffer[SBUS_DMA_BUFFER_SIZE] = {0}; // 50字节缓冲区
//SBUS_Data_t sbusData;

//////** #################################################### **//
//////** ================= 本地静态函数声明 ================= **//
//////** ################################################### **//

//static void HORRC_HT10A_GET_Ctl(SBUS_Data_t* RC_Raw);


//////** ############################################ **//
//////** ================= 对外函数 ================= **//
//////** ########################################### **//

//// 初始化函数
//void SBUS_Init(UART_HandleTypeDef *huart) {
//    sbusHuart = huart;
//    
//    // 清除旧数据
//    memset(sbusRxBuffer, 0, sizeof(sbusRxBuffer));
//    memset(&sbusData, 0, sizeof(sbusData));

//    // 开启DMA循环接收
//    // 只需要调用一次，DMA会一直在后台搬运数据到sbusRxBuffer
//    HAL_UART_Receive_DMA(sbusHuart, sbusRxBuffer, SBUS_FRAME_LEN);
//}

///**
//  * @brief  SBUS 数据解析函数 (包含自动同步逻辑)
//  *         建议在 main 循环中调用
//  * @retval None
//  */
//void SBUS_Parse(void) {
//    uint8_t i;
//    uint8_t found = 0;

//    // 1. 检查是否有新数据
//    if (sbusData.newDataAvailable == 0) return;

//		// 自动同步：在 50 字节缓冲区中寻找帧头 0x0F
//    for (i = 0; i <= (SBUS_DMA_BUFFER_SIZE - SBUS_FRAME_LEN); i++) {
//        if (sbusRxBuffer[i] == 0x0F) {
//            // 校验帧尾
//            if (sbusRxBuffer[i + 24] == 0x00) {
//                
//                // 找到完整帧，定义数据起始指针 (跳过帧头)
//                const uint8_t *d = &sbusRxBuffer[i + 1];

//                // === 标准 SBUS 11bit 通道解包公式 ===
//                // 使用显式 uint16_t 转换和完整括号，防止编译器优化截断
//                sbusData.channels[0]  = (((uint16_t)d[0] << 0) | ((uint16_t)d[1] << 8)) & 0x07FF;
//                sbusData.channels[1]  = (((uint16_t)d[1] >> 3) | ((uint16_t)d[2] << 5)) & 0x07FF;
//                sbusData.channels[2]  = (((uint16_t)d[2] >> 6) | ((uint16_t)d[3] << 2) | ((uint16_t)d[4] << 10)) & 0x07FF;
//                sbusData.channels[3]  = (((uint16_t)d[4] >> 1) | ((uint16_t)d[5] << 7)) & 0x07FF;
//                sbusData.channels[4]  = (((uint16_t)d[5] >> 4) | ((uint16_t)d[6] << 4)) & 0x07FF;
//                sbusData.channels[5]  = (((uint16_t)d[6] >> 7) | ((uint16_t)d[7] << 1) | ((uint16_t)d[8] << 9)) & 0x07FF;
//                sbusData.channels[6]  = (((uint16_t)d[8] >> 2) | ((uint16_t)d[9] << 6)) & 0x07FF;
//                sbusData.channels[7]  = (((uint16_t)d[9] >> 5) | ((uint16_t)d[10] << 3)) & 0x07FF;
//                
//                sbusData.channels[8]  = (((uint16_t)d[11] << 0) | ((uint16_t)d[12] << 8)) & 0x07FF;
//                sbusData.channels[9]  = (((uint16_t)d[12] >> 3) | ((uint16_t)d[13] << 5)) & 0x07FF;
//                sbusData.channels[10] = (((uint16_t)d[13] >> 6) | ((uint16_t)d[14] << 2) | ((uint16_t)d[15] << 10)) & 0x07FF;
//                sbusData.channels[11] = (((uint16_t)d[15] >> 1) | ((uint16_t)d[16] << 7)) & 0x07FF;
//                sbusData.channels[12] = (((uint16_t)d[16] >> 4) | ((uint16_t)d[17] << 4)) & 0x07FF;
//                sbusData.channels[13] = (((uint16_t)d[17] >> 7) | ((uint16_t)d[18] << 1) | ((uint16_t)d[19] << 9)) & 0x07FF;
//                sbusData.channels[14] = (((uint16_t)d[19] >> 2) | ((uint16_t)d[20] << 6)) & 0x07FF;
//                sbusData.channels[15] = (((uint16_t)d[20] >> 5) | ((uint16_t)d[21] << 3)) & 0x07FF;

//                // 解析数字通道和状态标志
//                sbusData.ch17      = (d[22] >> 0) & 0x01;
//                sbusData.ch18      = (d[22] >> 1) & 0x01;
//                sbusData.frameLost = (d[22] >> 2) & 0x01;
//                sbusData.failsafe  = (d[22] >> 3) & 0x01;

//                found = 1;
//                break;
//            }
//        }
//    }

//    // 6. 处理结果
//    if (found) {
//        // 解析成功，数据已更新到 sbusData 结构体中
//		HORRC_HT10A_GET_Ctl(&sbusData);
//    } else {
//			
//		// 清除旧数据
//		 sbusData.newDataAvailable = 0;
//    }
//    
//    // 清除标志位，准备接收下一次 DMA 完成通知
//    sbusData.newDataAvailable = 0;
//}


////** ############################################### **//
////** ================= 本地静态函数 ================= **//
////** ############################################### **//
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

static void HORRC_HT10A_GET_Ctl(SbusData_t* sbusData)
{
		RC_Ctl.Stick.LX = sbusData->channels[3] - HOTRC_MID_VEL;
		RC_Ctl.Stick.LY = sbusData->channels[2] - HOTRC_MID_VEL;
		RC_Ctl.Stick.RX = sbusData->channels[0] - HOTRC_MID_VEL;
		RC_Ctl.Stick.RY = sbusData->channels[1] - HOTRC_MID_VEL;
	
		RC_Ctl.Switch.S2_L = Switch_Set(sbusData->channels[5]);
		RC_Ctl.Switch.S2_R = Switch_Set(sbusData->channels[6]);
		RC_Ctl.Switch.S3_L = Switch_Set(sbusData->channels[4]);
		RC_Ctl.Switch.S3_R = Switch_Set(sbusData->channels[7]);
		
		RC_Ctl.Knob.KL = sbusData->channels[8];
	  RC_Ctl.Knob.KR = sbusData->channels[9];
	
		RC_Ctl.Flag.ch17 = sbusData->ch17;
		RC_Ctl.Flag.ch18 = sbusData->ch18;
		RC_Ctl.Flag.failsafe = sbusData->failsafe;
		RC_Ctl.Flag.frameLost = sbusData->frameLost;
}


// ================= 私有变量 =================
static UART_HandleTypeDef *sbusHuart;
static uint8_t sbusRxBuffer[SBUS_RX_BUF_SIZE]; // DMA 循环写入的缓冲区

// ================= 全局变量定义 =================
SbusData_t sbusData;

// ================= 私有函数前向声明 =================
static void SBUS_DecodeFrame(const uint8_t *frameData);

// ================= 初始化函数 =================
void SBUS_Init(UART_HandleTypeDef *huart) {
    sbusHuart = huart;
    
    // 清零数据
    memset(&sbusData, 0, sizeof(sbusData));
    memset(sbusRxBuffer, 0, sizeof(sbusRxBuffer));
    
    // 1. 开启 IDLE 中断 (使能串口空闲中断)
    // 注意：HAL库没有直接的宏，需要手动操作寄存器
    __HAL_UART_ENABLE_IT(sbusHuart, UART_IT_IDLE);
    
    // 2. 开启 DMA 循环接收
    // 使用 HAL_UARTEx_ReceiveToIdle_DMA 或者 HAL_UART_Receive_DMA (CIRCULAR模式)
    // 这里使用最通用的 HAL_UART_Receive_DMA，前提是在 CubeMX 中把 DMA Mode 设为 Circular
    HAL_UART_Receive_DMA(sbusHuart, sbusRxBuffer, SBUS_RX_BUF_SIZE);
}

// ================= 核心处理函数 (必须在 main 循环调用) =================
void SBUS_Process(void) {
    uint8_t found = 0;
    uint16_t i;
    
    // 触发条件：要么 IDLE 中断置了标志，要么每隔 5ms 强制扫一次 (容错机制)
    // 这里的 5ms 是容错时间，保证即使中断没触发，最多 5ms 也会主动去看一眼
    if (sbusData.newDataFlag || (HAL_GetTick() - sbusData.lastParseTick > 5)) {
        
        // 扫描整个缓冲区寻找合法的 SBUS 帧
        // 因为是循环缓冲区，我们线性扫描即可，DMA 会自动处理覆盖
        for (i = 0; i <= (SBUS_RX_BUF_SIZE - SBUS_FRAME_LEN); i++) {
            // 检查帧头 (0x0F) 和 帧尾 (0x00)
            if (sbusRxBuffer[i] == SBUS_HEADER && sbusRxBuffer[i + 24] == SBUS_FOOTER) {
                // 找到完整帧，进行解码
                SBUS_DecodeFrame(&sbusRxBuffer[i]);
                found = 1;
                break; // 找到一帧即可，若需要最新的，可以反向遍历
            }
        }
        
        // 清理工作
        sbusData.newDataFlag = 0;          // 清除中断标志
        sbusData.lastParseTick = HAL_GetTick(); // 更新时间戳
    }
}

// ================= 帧解码函数 (私有) =================
static void SBUS_DecodeFrame(const uint8_t *d) {
    // d 指向帧头 (0x0F)，我们从 d+1 开始解析数据
    
    // 通道 0-15 解包 (11bit per channel)
    sbusData.channels[0]  = (((uint16_t)d[1]  << 0)  | ((uint16_t)d[2]  << 8)) & 0x07FF;
    sbusData.channels[1]  = (((uint16_t)d[2]  >> 3)  | ((uint16_t)d[3]  << 5)) & 0x07FF;
    sbusData.channels[2]  = (((uint16_t)d[3]  >> 6)  | ((uint16_t)d[4]  << 2)  | ((uint16_t)d[5] << 10)) & 0x07FF;
    sbusData.channels[3]  = (((uint16_t)d[5]  >> 1)  | ((uint16_t)d[6]  << 7)) & 0x07FF;
    sbusData.channels[4]  = (((uint16_t)d[6]  >> 4)  | ((uint16_t)d[7]  << 4)) & 0x07FF;
    sbusData.channels[5]  = (((uint16_t)d[7]  >> 7)  | ((uint16_t)d[8]  << 1)  | ((uint16_t)d[9] << 9))  & 0x07FF;
    sbusData.channels[6]  = (((uint16_t)d[9]  >> 2)  | ((uint16_t)d[10] << 6)) & 0x07FF;
    sbusData.channels[7]  = (((uint16_t)d[10] >> 5)  | ((uint16_t)d[11] << 3)) & 0x07FF;
    
    sbusData.channels[8]  = (((uint16_t)d[12] << 0)  | ((uint16_t)d[13] << 8)) & 0x07FF;
    sbusData.channels[9]  = (((uint16_t)d[13] >> 3)  | ((uint16_t)d[14] << 5)) & 0x07FF;
    sbusData.channels[10] = (((uint16_t)d[14] >> 6)  | ((uint16_t)d[15] << 2)  | ((uint16_t)d[16] << 10)) & 0x07FF;
    sbusData.channels[11] = (((uint16_t)d[16] >> 1)  | ((uint16_t)d[17] << 7)) & 0x07FF;
    sbusData.channels[12] = (((uint16_t)d[17] >> 4)  | ((uint16_t)d[18] << 4)) & 0x07FF;
    sbusData.channels[13] = (((uint16_t)d[18] >> 7)  | ((uint16_t)d[19] << 1)  | ((uint16_t)d[20] << 9))  & 0x07FF;
    sbusData.channels[14] = (((uint16_t)d[20] >> 2)  | ((uint16_t)d[21] << 6)) & 0x07FF;
    sbusData.channels[15] = (((uint16_t)d[21] >> 5)  | ((uint16_t)d[22] << 3)) & 0x07FF;

    // 解析数字通道和状态位 (第23个字节，索引为22)
    sbusData.ch17      = (d[23] >> 0) & 0x01;
    sbusData.ch18      = (d[23] >> 1) & 0x01;
    sbusData.frameLost = (d[23] >> 2) & 0x01;
    sbusData.failsafe  = (d[23] >> 3) & 0x01;
		
		HORRC_HT10A_GET_Ctl(&sbusData);
}

//// ================= 中断回调处理 (需在 stm32f4xx_it.c 中调用) =================
//// 请把这个函数放到 USARTx_IRQHandler 里，或者使用 HAL 回调
//void SBUS_UART_IRQHandler(void) {
//    // 检查是否是 IDLE 中断触发
//    if (__HAL_UART_GET_FLAG(sbusHuart, UART_FLAG_IDLE) != RESET) {
//        // 清除 IDLE 标志 (读 SR 寄存器，再读 DR 寄存器，这是 HAL 库的标准清除方式)
//        // 注意：仅仅 __HAL_UART_CLEAR_IDLEFLAG 有时不够保险，配合下面的操作
//        (void)sbusHuart->Instance->SR;
//        (void)sbusHuart->Instance->DR;
//        
//        // 标记有新数据到达
//        sbusData.newDataFlag = 1;
//        
//        // 注意：这里不要停止 DMA！我们用的是 CIRCULAR 模式，让它一直跑
//    }
//}


