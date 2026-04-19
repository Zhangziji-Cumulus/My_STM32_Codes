#ifndef __HOTRC_HT10A_H
#define __HOTRC_HT10A_H

#include "main.h"
#include "usart.h"
#include "string.h"
#include <stdint.h>
#include <stdbool.h>

// 定义SBUS解析结果结构体
typedef struct {
    uint16_t channels[16];  // 16个模拟通道 (0~2047)
    bool ch17;              // 数字通道17 (0x80)
    bool ch18;              // 数字通道18 (0x40)
    bool frame_lost;        // 帧丢失标志 (0x20) -> 接收机红灯亮
    bool failsafe;          // 故障保护激活 (0x10)
    bool valid;             // 数据帧是否合法
} SbusFrame_t;

typedef struct{
	
	//摇杆
	struct{
		unsigned short LX;// 左摇杆 X 轴
		unsigned short LY;// 左摇杆 Y 轴
		unsigned short RX;// 右摇杆 X 轴
		unsigned short RY;// 右摇杆 Y 轴
	}Stick;
	
	struct{
		unsigned short S2_L;// 两段拨杆1
		unsigned short S2_R;// 两段拨杆2
		unsigned short S3_L;// 三段拨杆1
		unsigned short S3_R;// 三段拨杆2
	}Switch;
	
	struct{
		unsigned short KL;
		unsigned short KR;
	}Knob;

	struct{
		bool ch17;              // 数字通道17 (0x80)
    bool ch18;              // 数字通道18 (0x40)
    bool frame_lost;        // 帧丢失标志 (0x20) -> 接收机红灯亮
    bool failsafe;          // 故障保护激活 (0x10)
    bool valid;             // 数据帧是否合法
	}Flag;

}HOTRC_Ctl_t;


void Remote_ControlInit(void);
void HOTRC_CallBack(UART_HandleTypeDef *huart);

#endif
