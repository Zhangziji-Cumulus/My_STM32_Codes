#include "MY_Time.h"





/**
* @brief:自定定时器回调函数
* @param:
* @return:
**/

void MY_TIM2_Callback(void)
{
	HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,LED_Flash(100,1));
}
