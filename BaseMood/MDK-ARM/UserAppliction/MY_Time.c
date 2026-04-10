#include "MY_Time.h"

/**
* @brief:自定定时器回调函数
* @param:
* @return:
**/

float test6 = 100.0f;

void MY_TIM2_Callback(void)
{
		Motor_DJI_Angle_SingleContral(test6);
//	static uint16_t timecount = 0;
//	if(timecount > 0 && timecount < 2000)
//	{
//			Motor_DJI_Angle_SingleContral(-170.0f);
//	}
//	if(timecount > 2000 && timecount < 4000)
//	{
//			Motor_DJI_Angle_SingleContral(10.0f);
//			timecount = 0;
//	}
//	
//	timecount++;

	//Motor_DJI_Speed_SingleContral(500);
}
