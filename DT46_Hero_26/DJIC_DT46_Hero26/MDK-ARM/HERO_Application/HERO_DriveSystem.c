#include "HERO_DriveSystem.h"

extern HOTRC_Ctl_t RC_Ctl;
extern Dual_Board_Transmit_t DBT_RX;
extern DJI_MotorFeedback_t DJI_MFeedback[8];

float test;

Chassis_PID_t Chassis_PID;

//定义地盘数据
Mecanum_Data_t Chassis_MD;
Wheels_Data_t wheels;

void get_data(void);

void HERO_DriveSystem_Init(void)
{

}

__attribute__((used)) void HERODriveSystemTask(void *argument)
{

  for(;;)
  {
		get_data();
		Chassis_Mecanum_Calc(&Chassis_MD);
		
		Motor_DJI_Speed_SingleContral(Chassis_MD.FL.T_Speed);
		
		HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,LED_Flash(100,1));
		
    osDelay(1);
  }

}

void get_data(void)
{ 
	Chassis_MD.Target.Xv = DBT_RX.B2.RC_Ctl.Stick.LX;
	Chassis_MD.Target.Yv = DBT_RX.B2.RC_Ctl.Stick.LY;
	Chassis_MD.Target.Rv = DBT_RX.B2.RC_Ctl.Stick.RX;
	
	Chassis_MD.FL.C_Speed = DJI_MFeedback[0].speed_rpm;
	Chassis_MD.FL.C_Curr  = DJI_MFeedback[0].current_ma;

}