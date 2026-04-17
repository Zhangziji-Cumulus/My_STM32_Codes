#include "UAP_Init.h"
#include "HOTRC_HT10A.h"

void UAP_Init(void)
{
	Motor_CInit();
	Remote_ControlInit();
}
