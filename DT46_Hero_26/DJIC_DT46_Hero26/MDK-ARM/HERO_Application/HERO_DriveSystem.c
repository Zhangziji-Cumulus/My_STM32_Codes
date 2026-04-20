#include "HERO_DriveSystem.h"

extern HOTRC_Ctl_t RC_Ctl;
extern Dual_Board_Transmit_t DBT_RX;
extern DJI_MotorFeedback_t DJI_MFeedback[8];

//定义地盘数据
Mecanum_Data_t Chassis_MD;
Wheels_Data_t wheels;

void get_data(void);

void HERO_DriveSystem_Init(void)
{

}

static UBaseType_t remain_HEROSystemTask;
__attribute__((used)) void HEROSystemTask(void *argument)
{
  for(;;)
  {
		get_data();
		
		remain_HEROSystemTask = uxTaskGetStackHighWaterMark(NULL);
    osDelay(1);
  }
}

static UBaseType_t remain_HEROChassisTask;
__attribute__((used)) void HEROChassisTask(void *argument)
{
  for(;;)
  {
		Chassis_Mecanum_Calc(&Chassis_MD);	
		Motor_DJI_SpeedCtl_5_8(&hcan1,1.0f,Chassis_MD.FL.T_Speed,Chassis_MD.FL.T_Speed,Chassis_MD.FL.T_Speed,Chassis_MD.FL.T_Speed);
		
		remain_HEROChassisTask = uxTaskGetStackHighWaterMark(NULL);
    osDelay(1);
  }
}

static UBaseType_t remain_HEROGimbalTask;
__attribute__((used)) void HEROGimbalTask(void *argument)
{
  for(;;)
  {

		remain_HEROGimbalTask = uxTaskGetStackHighWaterMark(NULL);
    osDelay(1);
  }
}

static UBaseType_t remain_HEROShootingTask;
__attribute__((used)) void HEROShootingTask(void *argument)
{
  for(;;)
  {
		
		remain_HEROShootingTask = uxTaskGetStackHighWaterMark(NULL);
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