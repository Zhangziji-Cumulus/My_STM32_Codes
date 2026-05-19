#include "CMDCenter.h"

static CMD_t CMD;

//** ================================================================================ **//
//** ================================== 对外函数 ==================================== **//
//** ================================================================================ **//

void CMD_Center_Init(void)
{
    
}

//** ------------------------------------------------------------ **//
//** ===================== 获取数据（指针） ====================== **//
//** ------------------------------------------------------------ **//
const CMD_t* CMD_Get(void)
{
    return &CMD;
}

//** ================================================================================ **//
//** ============================== 更新控制变量任务 ================================= **//
//** ================================================================================ **//
static UBaseType_t remain_CMDUpdateTask;
__attribute__((used)) void CMDUpdateTask(void *argument)
{

  for(;;)
  {
    const RC_Ctl_t* RC_Ctl = get_RC_Ctl_point();
    const VideoTx_Ctrl_t *VideoTx_Data = get_VideoTx_Ctl_point();

    //判断有没有正确接受到数据、数据是否正常；不正常则设置为“停止模式”
    if(VideoTx_Data->is_valid == 1 || RC_Ctl->is_valid == 1)
    {
      //更新控制模式
      if (VideoTx_Data->is_valid == 1)
      {
        CMD.ctrl = KEYBOARD_MODE;
      }
      else if (RC_Ctl->is_valid == 1)
      {
        CMD.ctrl = REMOTE_MODE;
      }

      //遥控器
      if(CMD.ctrl == REMOTE_MODE)
      {
        //移动模式
        if(RC_Ctl->Switch.S2_L == HOTRC_SWITCH_UP)
        {
          CMD.ctrl = STOP_MODE;
        }

        if(RC_Ctl->Switch.S3_R == HOTRC_SWITCH_MID)
        {
          CMD.Move = Spin_CW;
        }
        else if(RC_Ctl->Switch.S3_R == HOTRC_SWITCH_DOWN)
        {
          CMD.Move = Spin_CCW;
        }
        else
        {
          CMD.Move = Normal;
        }

        //底盘移动指令
        CMD.Chassis.FB   = RC_Ctl->Stick.LY;
        CMD.Chassis.LR   = RC_Ctl->Stick.LX;
        CMD.Chassis.RO   = RC_Ctl->Stick.RX;
        //云台移动指令
        CMD.Gimbal.Yaw   = RC_Ctl->Stick.RX;
        CMD.Gimbal.Pitch = RC_Ctl->Stick.RY;

        //发射机构指令
        
        //发射
        if(RC_Ctl->Switch.S3_L == HOTRC_SWITCH_UP)
        {
          CMD.Shooting.Fire = ON;
        }
        else
        {
          CMD.Shooting.Fire = OFF;
        }
        //上弹
        if(RC_Ctl->Switch.S3_L == HOTRC_SWITCH_DOWN)
        {
          CMD.Shooting.Load = ON;
        }
        else
        {
          CMD.Shooting.Load = OFF;
        }
      }
      //键盘控制
      else if(CMD.ctrl == KEYBOARD_MODE)
      {
        CMD.Chassis.FB = VideoTx_Data->ch2  - VTX_CHANNEL_MID;
        CMD.Chassis.LR = VideoTx_Data->ch3  - VTX_CHANNEL_MID;
        CMD.Chassis.RO = VideoTx_Data->ch0  - VTX_CHANNEL_MID;

        CMD.Gimbal.Yaw = VideoTx_Data->ch0    -  VTX_CHANNEL_MID; 
        CMD.Gimbal.Pitch = VideoTx_Data->ch1  -  VTX_CHANNEL_MID;
      }
    }
    else
    {
      CMD.ctrl = STOP_MODE;
    }

    //=============================检测剩余栈=================================//
	  remain_CMDUpdateTask = uxTaskGetStackHighWaterMark(NULL);
    osDelay(1);
  }
}


