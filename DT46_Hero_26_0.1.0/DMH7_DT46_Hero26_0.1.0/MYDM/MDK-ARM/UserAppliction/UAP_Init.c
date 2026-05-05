#include "UAP_Init.h"
#include "DJI_Motor_Contral.h"
#include "HOTRC_HT10A.h"
#include "usart.h"

#include "FDCAN_PART.h"
#include "UAP_FreeRTOS.h"

void UAP_Init(void)
{
	fdcan_filter_init();
	Motor_Init();
	SBUS_Init(&huart5);
}
