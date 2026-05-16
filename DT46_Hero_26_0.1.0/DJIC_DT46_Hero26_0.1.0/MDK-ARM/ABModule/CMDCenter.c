#include "CMDCenter.h"

static CMD_t CMD;

void CMD_Center_Init(void)
{
    
}

//更新控制量
static UBaseType_t remain_CMDUpdateTask;
__attribute__((used)) void CMDUpdateTask(void *argument)
{

  for(;;)
  {
    RC_Ctl_t* RC_Ctl = get_RC_Ctl_point();
    VideoTx_Data_t *VideoTx_Data = get_VideoTx_Ctl_point();

    // CMD.ctrl 
    // CMD.Move


    CMD.Chassis.FB   = RC_Ctl->Stick.LY;
    CMD.Chassis.LR   = RC_Ctl->Stick.LX;
    CMD.Chassis.RO   = RC_Ctl->Stick.RX;

    CMD.Gimbal.Yaw   = RC_Ctl->Stick.RY;
    CMD.Gimbal.Pitch = RC_Ctl->Stick.RY;

    // CMD.Shooting.Fire
    // CMD.Shooting.Load

    //=============================检测剩余栈=================================//
	remain_CMDUpdateTask = uxTaskGetStackHighWaterMark(NULL);
    osDelay(1);
  }
}

//获取控制指令数据指针
const CMD_t* CMD_Get(void)
{
    return &CMD;
}

//设置控制模式函数
void Set_CMD_Ctrl()
{

}
//设置移动模式函数
void Set_CMD_Move()
{

}
//设置地盘移动指令函数
void Set_CMD_Chassis()
{


}
//设置云台移动指令函数
void Set_CMD_Gimbal()
{

}
//设置发射机构指令函数
void Set_CMD_Shooting()
{

}