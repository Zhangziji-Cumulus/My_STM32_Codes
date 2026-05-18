#include "Chassis_Task.h"


__weak void Chassis_Publish(void); 
__weak void Chassis_Init(void); //初始化函数
__weak void Chassis_HandleError(void);//异常处理函数
__weak void Chassis_SetMode(void);//设置模式
__weak void Chassis_Update(void);//更新状态函数
__weak void Chassis_Reference(void);
__weak void Chassis_Console(void);
__weak void Chassis_SendCmd(void);

static UBaseType_t remain_ChassisTask;
__attribute__((used)) void ChassisTask(void *argument)
{
  for(;;)
  {	

    //=============================== 剩余栈检测 ===============================//
	remain_ChassisTask = uxTaskGetStackHighWaterMark(NULL);
    osDelay(CHASSIS_TASK_TIME_MS);
  }
}


