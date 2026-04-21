#include "HERO_DriveSystem.h"
extern Dual_Board_Transmit_t DBT_RX;


extern HOTRC_Ctl_t RC_Ctl;
extern DJI_MotorFeedback_t DJI_MFeedback[8];
extern fp32 INS_angle[3];

//定义地盘数据
Mecanum_Data_t Chassis_MD;
Wheels_Data_t wheels;

//定义云台数据
Gimbal_Data_t GD;


void get_chassis_data(void);
void get_gimbal_data(void);



void HERO_DriveSystem_Init(void)
{

}

static UBaseType_t remain_HEROSystemTask;
__attribute__((used)) void HEROSystemTask(void *argument)
{
  for(;;)
  {	
		remain_HEROSystemTask = uxTaskGetStackHighWaterMark(NULL);
    osDelay(1);
  }
}

static UBaseType_t remain_HEROChassisTask;
__attribute__((used)) void HEROChassisTask(void *argument)
{
  for(;;)
  {
		get_chassis_data();
		
		Chassis_Mecanum_Calc(&Chassis_MD);	
		Motor_DJI_SpeedCtl_1_4(&hcan1,1.0f,Chassis_MD.FL.T_Speed,Chassis_MD.FL.T_Speed,Chassis_MD.FL.T_Speed,Chassis_MD.FL.T_Speed);
		
		remain_HEROChassisTask = uxTaskGetStackHighWaterMark(NULL);
    osDelay(1);
  }
}

static UBaseType_t remain_HEROGimbalTask;
__attribute__((used)) void HEROGimbalTask(void *argument)
{
  for(;;)
  {
		get_gimbal_data();
		
		HERO_Gimbal_YawStable(&GD,RC_Ctl.Stick.RX);
		Motor_DJI_Angle_SingleContral(GD.TAngle.Yaw);

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

void get_chassis_data(void)
{ 
	Chassis_MD.Target.Xv = RC_Ctl.Stick.LX;
	Chassis_MD.Target.Yv = RC_Ctl.Stick.LY; 
	Chassis_MD.Target.Rv = RC_Ctl.Stick.RX;
	
	Chassis_MD.FL.C_Speed = DJI_MFeedback[0].speed_rpm;
	Chassis_MD.FL.C_Curr  = DJI_MFeedback[0].current_ma;
}

void get_gimbal_data(void)
{ 
	//Chassis_MD.Target.Xv = RC_Ctl.Stick.LX;
	//Chassis_MD.Target.Yv = RC_Ctl.Stick.LY;
	//Chassis_MD.Target.Rv = RC_Ctl.Stick.RX;
	
	 Chassis_MD.FL.C_Speed = DJI_MFeedback[0].speed_rpm;
	 Chassis_MD.FL.C_Curr  = DJI_MFeedback[0].current_ma;
}