#include "UAP_Init.h"
#include "Motor_Contral.h"
#include "HOTRC_HT10A.h"

#include "CAN_PART.h"

void UAP_Init(void)
{
	can_filter_init();
	
	//CAN_Filter_AcceptAllID(&hcan1,0);
	//CAN_Filter_AcceptAllID(&hcan2,14);

	//can_filter_init();
	Motor_CInit();
	Remote_ControlInit();
}
