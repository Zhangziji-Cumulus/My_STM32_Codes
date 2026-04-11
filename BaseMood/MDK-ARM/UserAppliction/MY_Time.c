#include "MY_Time.h"

/**
* @brief:自定定时器回调函数
* @param:
* @return:
**/

float test6 = 0.0f;

void MY_TIM2_Callback(void)
{
	
	Motor_DJI_Angle_SingleContral(test6);
	
	static uint16_t timecount = 0;
	if(timecount > 2000)
	{
		test6 -= 66.0f;
		if(test6 > 180.0f)
		{
			test6 = -180.0f;
		}
		else if(test6 < -180.0f)
		{
			test6 = 180.0f;
		}
		timecount = 0;
	}
	timecount++;

}
