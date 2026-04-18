#include "Dual_board_Transmit.h"



extern float IMU_DegAngle[3];

float Board1_IMUDatas[3];


extern float DualBoard_ReceiveDataBuff[64];
float DualBoard_SendDataBuff[64];

void Dual_Board_MainSend(void)
{
	if(BOARD_ID == 1)
	{
		//** Send **//
		
		//** Receive **//
		
		//Board-1 IMU Datas Send
		DualBoard_SendDataBuff[0] = IMU_DegAngle[0];
		DualBoard_SendDataBuff[1] = IMU_DegAngle[1];
		DualBoard_SendDataBuff[2] = IMU_DegAngle[2];
		DualBoard_SendDataBuff[3] = 666.0f;
		
		CAN_SendFloatArray(&hcan2,DualBoard_SendDataBuff,5,TX_BASE_ID);
		
		//Receive
	}
}

void Dual_Board_ReceiveCallBack(void)
{
	if(BOARD_ID == 2)
	{
		//** Send **//
		
		//** Receive **//
		
		//** Remote Control data receive **//
		
		//** Board-1 IMU Datas Reveive **//
		Board1_IMUDatas[0] = DualBoard_ReceiveDataBuff[0];
		Board1_IMUDatas[1] = DualBoard_ReceiveDataBuff[1];
		Board1_IMUDatas[2] = DualBoard_ReceiveDataBuff[2];
		//** Other data **//
	}
}

