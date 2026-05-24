#include "Dual_board_Transmit.h"

#if(BOARD_MODE == BOARD_MODE_DUAL)




//接受数据缓冲区
float DualBoard_ReceiveDataBuff[64];
//发送数据缓冲区
float DualBoard_SendDataBuff[64];

static UBaseType_t remain_DBTT;
__attribute__((used)) void Dual_Board_Transmit_Task(void *argument)
{
  for(;;)
  {
		
		remain_DBTT = uxTaskGetStackHighWaterMark(NULL);
    osDelay(1);
  }
}


static void Dual_Board_Send(void)
{
	if(BOARD_ID == 1)
	{
        	

		//Receive
	}
	else if(BOARD_ID == 2)
	{
		
	}
}


#endif

