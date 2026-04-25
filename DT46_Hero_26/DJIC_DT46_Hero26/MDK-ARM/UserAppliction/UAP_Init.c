#include "UAP_Init.h"
#include "DJI_Motor_Contral.h"
#include "HOTRC_HT10A.h"
#include "usart.h"

#include "CAN_PART.h"
#include "UAP_FreeRTOS.h"

void UAP_Init(void)
{
	
	
	can_filter_init();
	
	//CAN_Filter_AcceptAllID(&hcan1,0);
	//CAN_Filter_AcceptAllID(&hcan2,14);

	//can_filter_init();
	Motor_Init();
	SBUS_Init(&huart3);
	UAP_FreeRTOS_Init();
}
