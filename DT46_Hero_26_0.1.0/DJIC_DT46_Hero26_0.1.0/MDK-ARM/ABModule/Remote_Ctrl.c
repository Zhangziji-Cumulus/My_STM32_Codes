#include "Remote_Ctrl.h"

//HOTRC遥控器
#if (REMOTE_CTRL_TYPE == REMOTE_HOTRC)

RC_Ctl_t RC_Ctl;

static void HORRC_HT10A_GET_Ctl(void)
{
        const SbusData_t *sbusData = get_remote_control_point();

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

/** 获取遥控器数据指针（只读） */
const RC_Ctl_t* get_RC_Ctl(void)
{
    return &RC_Ctl;
}
/* 获取遥控器数据 */
const RC_Ctl_t get_RC_Ctl(void)
{
    return RC_Ctl;
}

#endif

//DJI DT7遥控器
#if (REMOTE_CTRL_TYPE == REMOTE_DJI_DT7)



#endif

