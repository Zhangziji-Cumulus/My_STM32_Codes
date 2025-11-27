#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#include "stm32f4xx.h"                  // Device header
#include "main.h"
#include "usart.h"
#include "Dual_board_Transmit.h"

//** 定义拨杆开关 **//
#define RC_LEVER_UP 	1
#define RC_LEVER_MID 	3
#define RC_LEVER_DOWN 	2
/* 遥控器CH0-3的范围 */
#define RC_CH_RANGE		660
#define RC_KV_RANGE		660

typedef struct
{
	struct
	{ 
		unsigned short ch0;
		unsigned short ch1;
		unsigned short ch2;
		unsigned short ch3;
		unsigned char s1;
		unsigned char s2;
	}rc;
	
	struct 
	{
		unsigned short x;
		unsigned short y;
		unsigned short z;
		unsigned char press_l;
		unsigned char press_r;
	}mouse;
	
	struct
	{
		unsigned short v;
	}key;
}RC_Ctl_t;



void Remote_ControlInit(void);
void ReMote_Control_CallBack(void);

#endif
