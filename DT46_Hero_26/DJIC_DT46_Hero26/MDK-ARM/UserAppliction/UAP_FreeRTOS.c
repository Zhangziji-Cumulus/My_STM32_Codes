#include "UAP_FreeRTOS.h"


extern osThreadId_t HEROChassisHandle;
extern osThreadId_t HEROGimbalHandle;
extern osThreadId_t HEROShootingHandle;
extern osThreadId_t HERODialHandle;

extern osMessageQueueId_t Queue_DJI_MDHandle;
extern DJI_MotorFeedback_t DJI_MFeedback[8];
extern HOTRC_Ctl_t RC_Ctl;




static UBaseType_t remain_StartRealTime_TASK;
__attribute__((used)) void StartRealTime_TASK(void *argument)
{
	 remain_StartRealTime_TASK = uxTaskGetStackHighWaterMark(NULL);
	 // 定义要发送的结构体变量
	 buzzer_t *buzzer = get_buzzer_effect_point();
	 uint8_t last_switch = 0; // 记录上一次拨杆状态
	
	 for(;;)
  { 
		
			if(RC_Ctl.Switch.S2_R == HOTRC_SWITCH_DOWN)
			{
				buzzer->sound_effect = D_D_D_;
			}
			
			// 2. 状态变化才执行，防止抖动
			if(RC_Ctl.Switch.S2_L != last_switch)
			{
					last_switch = RC_Ctl.Switch.S2_L;

					if(RC_Ctl.Switch.S2_L == 3)  // 拨杆 = 运行
					{
							// 唤醒应用任务
							if(HEROChassisHandle != NULL)
									vTaskResume(HEROChassisHandle);
							if(HEROChassisHandle != NULL)
									vTaskResume(HEROGimbalHandle);
							if(HEROChassisHandle != NULL)
									vTaskResume(HEROShootingHandle);
							if(HEROChassisHandle != NULL)
									vTaskResume(HERODialHandle);
							
					}
					else  // 拨杆 = 停止
					{
							// 挂起应用任务
							if(HEROChassisHandle != NULL)
									vTaskSuspend(HEROChassisHandle);
							if(HEROChassisHandle != NULL)
									vTaskSuspend(HEROGimbalHandle);
							if(HEROChassisHandle != NULL)
									vTaskSuspend(HEROChassisHandle);
							if(HEROChassisHandle != NULL)
									vTaskSuspend(HEROShootingHandle);
							if(HEROChassisHandle != NULL)
									vTaskSuspend(HERODialHandle);
																																			
							DJI_MOTOR_STOP_ALL(&hcan1);
					}
			 }

				// 3. 任务延时（消抖+释放CPU）
				vTaskDelay(50);
   }
	
		
    osDelay(1);
 }

static UBaseType_t Start_DJI_RecieveData_TASK;
__attribute__((used)) void Start_DJI_RecieveData(void *argument)
{
	
  for(;;)
  { Start_DJI_RecieveData_TASK = uxTaskGetStackHighWaterMark(NULL);
		
		    // 检查是否有新数据
    if (sbusData.newDataAvailable) {
        // 解析数据
        SBUS_Parse();
    }
    osDelay(1);
  }
}

