#include "MY_Time.h"

/**
* @brief:自定定时器回调函数
* @param:
* @return:
**/
void MY_TIM2_Callback(void)
{
	Motor_DJI_Speed_SingleContral(500);
}
