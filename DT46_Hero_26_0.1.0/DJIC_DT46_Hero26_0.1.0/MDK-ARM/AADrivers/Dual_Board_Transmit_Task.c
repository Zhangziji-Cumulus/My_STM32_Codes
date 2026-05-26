#include "Dual_Board_Transmit_Task.h"


#if(BOARD_MODE == BOARD_MODE_DUAL)

//** #################################################################################################### **//
//** =========================================== 云台板发送 ============================================= **//
//** #################################################################################################### **//

#if(BOARD_ID == GIMBAL_BOARD)

static UBaseType_t remain_DualBoardTask;
__attribute__((used)) void DualBoardTask(void *argument)
{
    
  BoardTransmit_Gimbal_TX_t Tx  = {0};

  for(;;)
  {

	  Tx.CMD = *CMD_Get_point();
		
    DualBoard_SendStruct(&hcan2,TX_BASE_ID,&Tx,sizeof(Tx));

		//=============================== 剩余栈检测 ===============================//
		remain_DualBoardTask = uxTaskGetStackHighWaterMark(NULL);
    osDelay(10);
  }

}

#endif

//** #################################################################################################### **//
//** =========================================== 地盘板发送 ============================================= **//
//** #################################################################################################### **//

#if(BOARD_ID == CHASSIS_BOARD)

__attribute__((used)) void DualBoardTask(void *argument)
{

  BoardTransmit_Chassis_TX_t Tx  = {0};

  for(;;)
  {

	  Tx.test = 2;
		
    DualBoard_SendStruct(&hcan2,TX_BASE_ID,&Tx,sizeof(Tx));

    osDelay(1);
  }

}

#endif

#endif