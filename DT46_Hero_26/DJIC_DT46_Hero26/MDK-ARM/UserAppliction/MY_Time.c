#include "MY_Time.h"


#include "Dual_board_Transmit.h"




/**
* @brief:自定定时器回调函数
* @param:
* @return:
**/

void MY_TIM2_Callback(void)
{
	//Motor_DJI_Speed_SingleContral(test6);
	
	//Motor_DJI_Speed_SingleContral(500);
	
	static uint16_t timecount = 0;
	
//	Motor_DJI_Speed_SingleContral(500);
	
//	Dual_Board_MainSend();
	
//	if(timecount > 50)
//	{
//		test6 += 66.0f;
//		if(test6 > 180.0f)
//		{
//			test6 = -180.0f;
//		}
//		else if(test6 < -180.0f)
//		{
//			test6 = 180.0f;
//		}
//		timecount = 0;
//	}
	timecount++;

}
