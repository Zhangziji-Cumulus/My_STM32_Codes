#include "ReSet_ToInit.h"

uint8_t ReSetFlag = 0;
uint8_t ReSetState = 1;
uint8_t ProgramState = 0;//1 is going and 0 is Stop 
extern RC_Ctl_t RC_Ctl;

extern int16_t F_L_MVel;
extern int16_t F_R_MVel;
extern int16_t B_L_MVel;
extern int16_t B_R_MVel;

extern float FWS_OutPutR;
extern float FWS_OutPutL;
extern float FWS_OutPutD;

extern int16_t Yaw_Vel;
extern int16_t Pitch_Vel;
extern int16_t Dial_Vel;


void UserInit(void)
{
	My_CAN_Init();
	Chassis_Init();
	Gimbal_Init();
	Shooting_Init();
	CAN_cmd_chassis(0,0,0,0);
	CAN_cmd_gimbal(0,0,0,0);
}

void GetReSetFlag(void)
{
	int8_t TempFlag = RC_Ctl.rc.s1;
	if((TempFlag == 3 || TempFlag == 2))
	{
		ReSetFlag = 1;
		ProgramState = 1;
	}
	else if(TempFlag == 1)
	{
		HAL_Delay(20);
			
		CAN_cmd_chassis(0,0,0,0);
		CAN_cmd_gimbal(0,0,0,0);
		
		HAL_Delay(50);
		ReSetFlag = 0;
		ReSetState = 0;
		ProgramState = 0;
	}
}

void ReSet_ToInit(void)
{
	if(ReSetFlag)
	{	 
		if(ReSetState == 0)
		{
			HAL_NVIC_SystemReset();
			ReSetState++;
		}
	}
}

void ReSet_LEDTip(void)
{
	HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_10);
	HAL_Delay(200);
	HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_10);
	HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11);
	HAL_Delay(200);
	HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11);
	HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12);
	HAL_Delay(200);
	HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12);
	HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11);
	HAL_Delay(200);
	HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11);
	HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_10);
	HAL_Delay(200);
	HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_10);
	HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11);
	HAL_Delay(200);
	HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11);
	HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12);
	HAL_Delay(200);
	HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12);
}


