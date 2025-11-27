#include "Dual_board_Transmit.h"

extern RC_Ctl_t RC_Ctl;
extern motor_measure_t motor_chassis[7];
extern float IMU_DegAngle[3];
extern GimBal_Variable_t GB_Varia;
extern float RxInitYawFlag;


float Board1_IMUDatas[3];


extern float DualBoard_ReceiveData[64];
float DualBoard_SendData[64];


extern uint8_t BIM088_ReSetFlag;

void Dual_Board_Send(void)
{
	if(BOARD_ID == 1)
	{
		//Send
		//Remote Control data Send
		DualBoard_SendData[0] = RC_Ctl.rc.ch0;
		DualBoard_SendData[1] = RC_Ctl.rc.ch1;
		DualBoard_SendData[2] = RC_Ctl.rc.ch2;
		DualBoard_SendData[3] = RC_Ctl.rc.ch3;
		DualBoard_SendData[4] = RC_Ctl.rc.s1;
		DualBoard_SendData[5] = RC_Ctl.rc.s2;
		DualBoard_SendData[6] = RC_Ctl.mouse.press_l;
		DualBoard_SendData[7] = RC_Ctl.mouse.press_r;
		DualBoard_SendData[8] = RC_Ctl.mouse.x;
		DualBoard_SendData[9] = RC_Ctl.mouse.y;
		DualBoard_SendData[10] = RC_Ctl.mouse.z;
		DualBoard_SendData[11] = RC_Ctl.key.v;
		//Board-1 IMU Datas Send
		DualBoard_SendData[12] = IMU_DegAngle[0];
		DualBoard_SendData[13] = IMU_DegAngle[1];
		DualBoard_SendData[14] = IMU_DegAngle[2];
		
		DualBoard_SendData[15] = BIM088_ReSetFlag;
		
		CAN_SendFloatArray(&hcan2,DualBoard_SendData,17);
		
		//Receive
		RxInitYawFlag = DualBoard_ReceiveData[0];
	}
}

void Dual_Board_ReceiveCallBack(void)
{
	if(BOARD_ID == 2)
	{
		//Send
		DualBoard_SendData[0] = GB_Varia.Yaw_InitFlag;
		CAN_SendFloatArray(&hcan2,DualBoard_SendData,1);
		
		//Receive
		//Remote Control data receive
		RC_Ctl.rc.ch0 = DualBoard_ReceiveData[0];
		RC_Ctl.rc.ch1 = DualBoard_ReceiveData[1];
		RC_Ctl.rc.ch2 = DualBoard_ReceiveData[2];
		RC_Ctl.rc.ch3 = DualBoard_ReceiveData[3];
		RC_Ctl.rc.s1 = DualBoard_ReceiveData[4];
		RC_Ctl.rc.s2 = DualBoard_ReceiveData[5];
		RC_Ctl.mouse.press_l  = DualBoard_ReceiveData[6];
		RC_Ctl.mouse.press_r = DualBoard_ReceiveData[7];
		RC_Ctl.mouse.x  = DualBoard_ReceiveData[8];
		RC_Ctl.mouse.y = DualBoard_ReceiveData[9];
		RC_Ctl.mouse.z = DualBoard_ReceiveData[10];
		RC_Ctl.key.v = DualBoard_ReceiveData[11];
		//Board-1 IMU Datas Reveive
		Board1_IMUDatas[0] = DualBoard_ReceiveData[12];
		Board1_IMUDatas[1] = DualBoard_ReceiveData[13];
		Board1_IMUDatas[2] = DualBoard_ReceiveData[14];
		BIM088_ReSetFlag = DualBoard_ReceiveData[15];
		
		//Other data
	}
}

