#include "Dual_board_Transmit.h"



extern float IMU_DegAngle[3];

float Board1_IMUDatas[3];


extern float DualBoard_ReceiveData[64];
float DualBoard_SendData[64];


extern uint8_t BIM088_ReSetFlag;

void Dual_Board_Send(void)
{
	if(BOARD_ID == 1)
	{
		//Send
		
		//Board-1 IMU Datas Send
		DualBoard_SendData[0] = IMU_DegAngle[0];
		DualBoard_SendData[1] = IMU_DegAngle[1];
		DualBoard_SendData[2] = IMU_DegAngle[2];
		
		CAN_SendFloatArray(&hcan2,DualBoard_SendData,4);
		
		//Receive
	}
}

void Dual_Board_ReceiveCallBack(void)
{
	if(BOARD_ID == 2)
	{
		//Send

		
		//Receive
		//Remote Control data receive
		
		//Board-1 IMU Datas Reveive
		Board1_IMUDatas[0] = DualBoard_ReceiveData[0];
		Board1_IMUDatas[1] = DualBoard_ReceiveData[1];
		Board1_IMUDatas[2] = DualBoard_ReceiveData[2];


		//Other data
	}
}

