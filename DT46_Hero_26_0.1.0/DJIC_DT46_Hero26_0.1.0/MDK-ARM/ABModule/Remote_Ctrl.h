#ifndef __REMOTE_CTRL_H__
#define __REMOTE_CTRL_H__

/* system includes */
#include "main.h"
/* user includes */
#include "bsp_SBUS.h"

//选择遥控器类型
#define REMOTE_HOTRC            0
#define REMOTE_DJI_DT7          1

#define REMOTE_CTRL_TYPE        REMOTE_HOTRC

//HOTRC遥控器
#if (REMOTE_CTRL_TYPE == REMOTE_HOTRC)

#define HOTRC_MID_VEL 992.0f //接受到的数据的正中值
#define HOTRC_RANGE 600

#define HOTRC_SWITCH_UP    1
#define HOTRC_SWITCH_MID   2
#define HOTRC_SWITCH_DOWN  3

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
	}Flag;

}RC_Ctl_t;

#endif

//DJI DT7遥控器
#if (REMOTE_CTRL_TYPE == REMOTE_DJI_DT7)



#endif

#endif /* __REMOTE_CTRL_H__ */