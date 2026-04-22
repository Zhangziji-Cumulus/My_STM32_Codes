#ifndef __HOTRC_HT10A_H
#define __HOTRC_HT10A_H

#include "main.h"
#include "usart.h"
#include "string.h"
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include <stdint.h>

#define HOTRC_MID_VEL 992.0f //接受到的数据的正中值
#define HOTRC_RANGE 600

#define HOTRC_SWITCH_UP    1
#define HOTRC_SWITCH_MID   2
#define HOTRC_SWITCH_DOWN  3

/* 定义缓冲区大小 */
/* 设置为 50 字节（2帧长度），这是实现自动同步的关键。
   如果缓冲区只有25字节，当发生错位时，DMA填满缓冲区可能刚好把一帧切成两半，导致永远解析不出完整数据。
   50字节保证缓冲区里至少躺着完整的一帧。 */
#define SBUS_DMA_BUFFER_SIZE 50 
// SBUS帧长度
#define SBUS_FRAME_LEN 25
// SBUS通道数
#define SBUS_CHANNEL_NUM 16



// SBUS数据结构体
typedef struct {
    uint16_t channels[SBUS_CHANNEL_NUM]; // 16个通道数据 (0-2047)
    uint8_t ch17;                        // 数字通道1 (0或1)
    uint8_t ch18;                        // 数字通道2 (0或1)
    uint8_t frameLost;                   // 丢帧标志 (1表示丢帧)
    uint8_t failsafe;                    // 故障安全标志 (1表示触发)
    uint8_t newDataAvailable;            // 新数据标志 (1表示有新一帧数据)
} SBUS_Data_t;

typedef struct{
	
	//摇杆
	struct{
		short LX;// 左摇杆 X 轴
		short LY;// 左摇杆 Y 轴
		short RX;// 右摇杆 X 轴
		short RY;// 右摇杆 Y 轴
	}Stick;
	
	struct{
		short S2_L;// 两段拨杆1
		short S2_R;// 两段拨杆2
		short S3_L;// 三段拨杆1
		short S3_R;// 三段拨杆2
	}Switch;
	
	struct{
		short KL;
		short KR;
	}Knob;

	struct{
    uint8_t ch17;                        // 数字通道1 (0或1)
    uint8_t ch18;                        // 数字通道2 (0或1)
    uint8_t frameLost;                   // 丢帧标志 (1表示丢帧)
    uint8_t failsafe;                    // 故障安全标志 (1表示触发)
    uint8_t newDataAvailable;            // 新数据标志 (1表示有新一帧数据)
	}Flag;

}HOTRC_Ctl_t;

/* 全局变量声明 */
extern uint8_t sbusRxBuffer[SBUS_DMA_BUFFER_SIZE];
extern SBUS_Data_t sbusData;

void SBUS_Init(UART_HandleTypeDef *huart);
void SBUS_Parse(void);


//void HOTRC_CallBack(UART_HandleTypeDef *huart);
//void HOTRC_ErrorCallback(UART_HandleTypeDef *huart);

#endif
