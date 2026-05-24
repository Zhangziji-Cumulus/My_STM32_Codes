#include "Dual_board_Transmit.h"

#if(BOARD_MODE == BOARD_MODE_DUAL)

Dual_Board_Transmit_t DBT;


//接受数据缓冲区
float DualBoard_ReceiveDataBuff[64];
//发送数据缓冲区
float DualBoard_SendDataBuff[64];

static UBaseType_t remain_DBTT;
__attribute__((used)) void Dual_Board_Transmit_Task(void *argument)
{
  for(;;)
  {
		
		Dual_Board_MainSend();
		
		remain_DBTT = uxTaskGetStackHighWaterMark(NULL);
    osDelay(1);
  }
}


static void Dual_Board_Send(void)
{
	if(BOARD_ID == 1)
	{
        

		//** Send **//
		DualBoard_SendDataBuff[0] = DBT.Tx.CMD.Chassis.FB;

		
		CAN_SendFloatArray(&hcan2,DualBoard_SendDataBuff,(14 + 1),TX_BASE_ID);
		//Receive
	}
	else if(BOARD_ID == 2)
	{
		
	}
}


#endif

