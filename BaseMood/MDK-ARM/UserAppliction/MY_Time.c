#include "MY_Time.h"

/**
* @brief:自定定时器回调函数
* @param:
* @return:
**/
void MY_TIM2_Callback(void)
{
			float test[4] = { 0.0f , -0.5f , 1.0f ,0.0f};
			ESC_Control_Amps_Group(&hcan1,&ESC_C610_10A,1,test);
}
