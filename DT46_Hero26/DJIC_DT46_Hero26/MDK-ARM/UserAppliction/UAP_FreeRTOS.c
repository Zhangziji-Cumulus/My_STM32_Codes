#include "UAP_FreeRTOS.h"
//#include "freertos.h"

extern DJI_MotorFeedback_t DJI_MFeedback[8];
extern osMessageQueueId_t Queue_DJI_MDHandle;

__attribute__((used)) void StartRealTime_TASK(void *argument)
{
	 // 定义要发送的结构体变量
   DJI_MotorFeedback_t feedback_data;
	 static uint8_t timecount = 0;
	 for(;;)
  {		
				if(timecount >= 8)
				{
						timecount = 0;
				}
		
			 BaseType_t ret = osMessageQueuePut(
            Queue_DJI_MDHandle,
            &DJI_MFeedback[timecount],    // 直接传结构体地址
            0,
            pdMS_TO_TICKS(10)
       );

    if (ret != osOK) {
            // 队列满，发送失败
    }
		
		timecount++;
    osDelay(10);
  }
}
DJI_MotorFeedback_t recv_data_buff;
DJI_MotorFeedback_t recv_data[8];

__attribute__((used)) void Start_DJI_RecieveData(void *argument)
{
	
  for(;;)
  {
		// 1. 阻塞等待队列消息
        BaseType_t ret = osMessageQueueGet(
            Queue_DJI_MDHandle,
            &recv_data_buff,       // 数据会被完整拷贝到这里
            NULL,
            osWaitForever     // 永久等待
        );
				recv_data[recv_data_buff.id - 1] = recv_data_buff;
        // 2. 接收成功 → 直接使用结构体成员
        if (ret == osOK) {

        }
		
    osDelay(1);
  }
}

