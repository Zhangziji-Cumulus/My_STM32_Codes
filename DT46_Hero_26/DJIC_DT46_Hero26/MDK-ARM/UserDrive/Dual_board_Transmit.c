#include "Dual_board_Transmit.h"

extern HOTRC_Ctl_t RC_Ctl;
extern float IMU_DegAngle[3];

Dual_Board_Transmit_t DBT_RX;

float DualBoard_ReceiveDataBuff[64];
float DualBoard_SendDataBuff[64];

static void Dual_Board_MainSend(void);

__attribute__((used)) void Dual_Board_Transmit_Task(void *argument)
{
  /* USER CODE BEGIN Dual_Board_Transmit_Task */
  /* Infinite loop */
	
  for(;;)
  {
		Dual_Board_MainSend();
		
    osDelay(1);
  }
  /* USER CODE END Dual_Board_Transmit_Task */
}

void Dual_Board_ReceiveCallBack(void)
{
	if(BOARD_ID == 1)
	{
		
	}
	else if(BOARD_ID == 2)
	{
		//** Send **//
		
		//** Receive **//
		
		//** Remote Control data receive **//
		
		//** Board-1 IMU Datas Reveive **//
		DBT_RX.B2.IMU[0] = DualBoard_ReceiveDataBuff[0];
		DBT_RX.B2.IMU[1] = DualBoard_ReceiveDataBuff[1];
		DBT_RX.B2.IMU[2] = DualBoard_ReceiveDataBuff[2];
		
		DBT_RX.B2.RC_Ctl.Stick.LX = DualBoard_ReceiveDataBuff[3];
		DBT_RX.B2.RC_Ctl.Stick.LY = DualBoard_ReceiveDataBuff[4];
		DBT_RX.B2.RC_Ctl.Stick.RX = DualBoard_ReceiveDataBuff[5];
		DBT_RX.B2.RC_Ctl.Stick.LY = DualBoard_ReceiveDataBuff[6];
		
		DBT_RX.B2.RC_Ctl.Switch.S2_L = DualBoard_ReceiveDataBuff[7];
		DBT_RX.B2.RC_Ctl.Switch.S2_R = DualBoard_ReceiveDataBuff[8];
		DBT_RX.B2.RC_Ctl.Switch.S3_L = DualBoard_ReceiveDataBuff[9];
		DBT_RX.B2.RC_Ctl.Switch.S3_R = DualBoard_ReceiveDataBuff[10];
		
		DBT_RX.B2.RC_Ctl.Knob.KL = DualBoard_ReceiveDataBuff[11];
		DBT_RX.B2.RC_Ctl.Knob.KR = DualBoard_ReceiveDataBuff[12];
		//** Other data **//
	}
}

static void Dual_Board_MainSend(void)
{
	if(BOARD_ID == 1)
	{
		//** Send **//
				
		//Board-1 IMU Datas Send
		DualBoard_SendDataBuff[0] = IMU_DegAngle[0];
		DualBoard_SendDataBuff[1] = IMU_DegAngle[1];
		DualBoard_SendDataBuff[2] = IMU_DegAngle[2];
		
		DualBoard_SendDataBuff[3] = RC_Ctl.Stick.LX;
		DualBoard_SendDataBuff[4] = RC_Ctl.Stick.LY;
		DualBoard_SendDataBuff[5] = RC_Ctl.Stick.RX;
		DualBoard_SendDataBuff[6] = RC_Ctl.Stick.LY;
		
		DualBoard_SendDataBuff[7]  = RC_Ctl.Switch.S2_L;
		DualBoard_SendDataBuff[8]  = RC_Ctl.Switch.S2_R;
		DualBoard_SendDataBuff[9]  = RC_Ctl.Switch.S3_L;
		DualBoard_SendDataBuff[10] = RC_Ctl.Switch.S3_R;
		
		DualBoard_SendDataBuff[11] = RC_Ctl.Knob.KL;
		DualBoard_SendDataBuff[12] = RC_Ctl.Knob.KR;
		
		CAN_SendFloatArray(&hcan2,DualBoard_SendDataBuff,(13 + 1),TX_BASE_ID);
		//Receive
	}
	else if(BOARD_ID == 2)
	{
		
	}
}





