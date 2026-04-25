#include "HOTRC_HT10A.h"
#include "string.h"

//接受结构体
HOTRC_Ctl_t RC_Ctl;

// 全局变量
UART_HandleTypeDef *sbusHuart;
uint8_t sbusRxBuffer[SBUS_DMA_BUFFER_SIZE] = {0}; // 50字节缓冲区
SBUS_Data_t sbusData;

////** #################################################### **//
////** ================= 本地静态函数声明 ================= **//
////** ################################################### **//

static void HORRC_HT10A_GET_Ctl(SBUS_Data_t* RC_Raw);


////** ############################################ **//
////** ================= 对外函数 ================= **//
////** ########################################### **//

// 初始化函数
void SBUS_Init(UART_HandleTypeDef *huart) {
    sbusHuart = huart;
    
    // 清除旧数据
    memset(sbusRxBuffer, 0, sizeof(sbusRxBuffer));
    memset(&sbusData, 0, sizeof(sbusData));

    // 开启DMA循环接收
    // 只需要调用一次，DMA会一直在后台搬运数据到sbusRxBuffer
    HAL_UART_Receive_DMA(sbusHuart, sbusRxBuffer, SBUS_FRAME_LEN);
}

/**
  * @brief  SBUS 数据解析函数 (包含自动同步逻辑)
  *         建议在 main 循环中调用
  * @retval None
  */
void SBUS_Parse(void) {
    uint8_t i;
    uint8_t found = 0;

    // 1. 检查是否有新数据
    if (sbusData.newDataAvailable == 0) return;

		// 自动同步：在 50 字节缓冲区中寻找帧头 0x0F
    for (i = 0; i <= (SBUS_DMA_BUFFER_SIZE - SBUS_FRAME_LEN); i++) {
        if (sbusRxBuffer[i] == 0x0F) {
            // 校验帧尾
            if (sbusRxBuffer[i + 24] == 0x00) {
                
                // 找到完整帧，定义数据起始指针 (跳过帧头)
                const uint8_t *d = &sbusRxBuffer[i + 1];

                // === 标准 SBUS 11bit 通道解包公式 ===
                // 使用显式 uint16_t 转换和完整括号，防止编译器优化截断
                sbusData.channels[0]  = (((uint16_t)d[0] << 0) | ((uint16_t)d[1] << 8)) & 0x07FF;
                sbusData.channels[1]  = (((uint16_t)d[1] >> 3) | ((uint16_t)d[2] << 5)) & 0x07FF;
                sbusData.channels[2]  = (((uint16_t)d[2] >> 6) | ((uint16_t)d[3] << 2) | ((uint16_t)d[4] << 10)) & 0x07FF;
                sbusData.channels[3]  = (((uint16_t)d[4] >> 1) | ((uint16_t)d[5] << 7)) & 0x07FF;
                sbusData.channels[4]  = (((uint16_t)d[5] >> 4) | ((uint16_t)d[6] << 4)) & 0x07FF;
                sbusData.channels[5]  = (((uint16_t)d[6] >> 7) | ((uint16_t)d[7] << 1) | ((uint16_t)d[8] << 9)) & 0x07FF;
                sbusData.channels[6]  = (((uint16_t)d[8] >> 2) | ((uint16_t)d[9] << 6)) & 0x07FF;
                sbusData.channels[7]  = (((uint16_t)d[9] >> 5) | ((uint16_t)d[10] << 3)) & 0x07FF;
                
                sbusData.channels[8]  = (((uint16_t)d[11] << 0) | ((uint16_t)d[12] << 8)) & 0x07FF;
                sbusData.channels[9]  = (((uint16_t)d[12] >> 3) | ((uint16_t)d[13] << 5)) & 0x07FF;
                sbusData.channels[10] = (((uint16_t)d[13] >> 6) | ((uint16_t)d[14] << 2) | ((uint16_t)d[15] << 10)) & 0x07FF;
                sbusData.channels[11] = (((uint16_t)d[15] >> 1) | ((uint16_t)d[16] << 7)) & 0x07FF;
                sbusData.channels[12] = (((uint16_t)d[16] >> 4) | ((uint16_t)d[17] << 4)) & 0x07FF;
                sbusData.channels[13] = (((uint16_t)d[17] >> 7) | ((uint16_t)d[18] << 1) | ((uint16_t)d[19] << 9)) & 0x07FF;
                sbusData.channels[14] = (((uint16_t)d[19] >> 2) | ((uint16_t)d[20] << 6)) & 0x07FF;
                sbusData.channels[15] = (((uint16_t)d[20] >> 5) | ((uint16_t)d[21] << 3)) & 0x07FF;

                // 解析数字通道和状态标志
                sbusData.ch17      = (d[22] >> 0) & 0x01;
                sbusData.ch18      = (d[22] >> 1) & 0x01;
                sbusData.frameLost = (d[22] >> 2) & 0x01;
                sbusData.failsafe  = (d[22] >> 3) & 0x01;

                found = 1;
                break;
            }
        }
    }

    // 6. 处理结果
    if (found) {
        // 解析成功，数据已更新到 sbusData 结构体中
		HORRC_HT10A_GET_Ctl(&sbusData);
    } else {
			
		// 清除旧数据
		 sbusData.newDataAvailable = 0;
     memset(sbusRxBuffer, 0, sizeof(sbusRxBuffer));
     memset(&sbusData, 0, sizeof(sbusData));

    }
    
    // 清除标志位，准备接收下一次 DMA 完成通知
    sbusData.newDataAvailable = 0;
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

static void HORRC_HT10A_GET_Ctl(SBUS_Data_t* RC_Raw)
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
		RC_Ctl.Flag.frameLost = RC_Raw->frameLost;
		RC_Ctl.Flag.newDataAvailable = RC_Raw->newDataAvailable;
}







