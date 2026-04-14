#include "UAP_FreeRTOS.h"
#include "freertos.h"

//extern DJI_MotorFeedback_t DJI_MFeedback[8];
extern osMessageQueueId_t Queue_DJI_MDHandle;

__attribute__((used)) void StartRealTime_TASK(void *argument)
{
	 // 定义要发送的结构体变量
   DJI_MotorFeedback_t feedback_data;
	 for(;;)
  {
		   // 1. 填充数据（CAN 解码/读取电机数据后赋值）
       feedback_data.id = 1;
       feedback_data.angle_raw = 4096;
       feedback_data.angle_deg = 180.0f;
       feedback_data.speed_rpm = 300;
       feedback_data.current_ma = 1500;
       feedback_data.error_code = 0;
       feedback_data.is_online = true;
			// 2. 发送到队列（最多等 10ms）
     BaseType_t ret = osMessageQueuePut(
            Queue_DJI_MDHandle,
            &feedback_data,    // 直接传结构体地址
            0,
            pdMS_TO_TICKS(10)
       );

    if (ret != osOK) {
            // 队列满，发送失败
    }
		
    osDelay(10);
  }
}

DJI_MotorFeedback_t recv_data;
__attribute__((used)) void Start_DJI_RecieveData(void *argument)
{
	
  for(;;)
  {
		// 1. 阻塞等待队列消息
        BaseType_t ret = osMessageQueueGet(
            Queue_DJI_MDHandle,
            &recv_data,       // 数据会被完整拷贝到这里
            NULL,
            pdMS_TO_TICKS(10) //osWaitForever     // 永久等待
        );

        // 2. 接收成功 → 直接使用结构体成员
        if (ret == osOK) {

        }
		
    osDelay(1);
  }
}

