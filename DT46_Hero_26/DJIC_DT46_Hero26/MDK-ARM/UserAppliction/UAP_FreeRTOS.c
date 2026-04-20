#include "UAP_FreeRTOS.h"

//蜂鸣器
extern DJI_MotorFeedback_t DJI_MFeedback[8];
extern osMessageQueueId_t Queue_DJI_MDHandle;
extern HOTRC_Ctl_t RC_Ctl;

static UBaseType_t remain_StartRealTime_TASK;
__attribute__((used)) void StartRealTime_TASK(void *argument)
{
	 // 定义要发送的结构体变量
	 buzzer_t *buzzer = get_buzzer_effect_point();
	
	 for(;;)
  {					
		if(RC_Ctl.Switch.S2_L == 1)
		{
			buzzer->sound_effect = D_D_D_;
		}
		
		remain_StartRealTime_TASK = uxTaskGetStackHighWaterMark(NULL);
    osDelay(1);
  }
}

__attribute__((used)) void Start_DJI_RecieveData(void *argument)
{
	
  for(;;)
  {
		
    osDelay(1);
  }
}

