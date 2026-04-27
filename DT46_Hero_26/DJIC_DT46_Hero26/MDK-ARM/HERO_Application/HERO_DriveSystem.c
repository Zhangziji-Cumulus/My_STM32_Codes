#include "HERO_DriveSystem.h"
#include "HERO_API.h"

extern Dual_Board_Transmit_t DBT_RX;

extern HOTRC_Ctl_t RC_Ctl;
extern DJI_MotorFeedback_t DJI_MFeedback_CAN1[8];
extern DJI_MotorFeedback_t DJI_MFeedback_CAN2[8];

extern fp32 INS_angle[3];
extern float IMU_DegAngle[3];

// 全局标志：0=停止电机  1=正常运行
extern uint8_t g_motor_run_enable;

//定义地盘数据
extern PID_HandleTypeDef Chassis_Follow_PID;
Mecanum_Data_t Chassis_Mec;
Chassis_Move_t Chassis_Move;
Wheels_Data_t wheels;

//定义云台数据
Gimbal_Data_t GD;

//定义摩擦轮数据
ShootingVel_t SV;


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
		static uint16_t timecount2 = 200;
		
		if(g_motor_run_enable == 0)
		{
			//电机急刹车
			if(timecount2 < 150)
			{
				DJI_MOTOR_EmergencySTOP_ALL(DJI_MFeedback_CAN1,&hcan1,10.0f);
				timecount2++;
			}
		}
		else
		{	
			timecount2 = 0;
			Chassis_Move_Calc(&Chassis_Move,&Chassis_Mec,&Chassis_Follow_PID,RC_Ctl.Switch.S3_R);
			Motor_DJI_Chassis(DJI_MFeedback_CAN1,&hcan1,1.0f,Chassis_Mec.FL.T_rpm,Chassis_Mec.FR.T_rpm,Chassis_Mec.BL.T_rpm,Chassis_Mec.BR.T_rpm);	
		}
//==============================================================//
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
		
		static uint16_t timecount1 = 0;
		
		if(g_motor_run_enable == 0)
		{
			//电机急刹车
			if(timecount1 < 150)
			{
				DJI_MOTOR_EmergencySTOP_ALL(DJI_MFeedback_CAN2,&hcan2,10.0f);
				timecount1++;
			}
		}
		else
		{		
			timecount1 = 0;
			if(abs(DJI_MFeedback_CAN2[0].current_ma) < 1000)
			{
				HERO_Gimbal_PitchStable(&GD,RC_Ctl.Stick.RY);
				Motor_DJI_IMUPitchContral(DJI_MFeedback_CAN2,GD.TAngle.Pitch,-(IMU_DegAngle[2]),1,1);
			}
			if(DJI_MFeedback_CAN2[3].current_ma < 1000)
			{
				HERO_Gimbal_YawStable(&GD,RC_Ctl.Stick.RX);
				Motor_DJI_IMUYawContral(DJI_MFeedback_CAN2,GD.TAngle.Yaw,IMU_DegAngle[0],5,2);
			}
		}
//==============================================================//
		remain_HEROGimbalTask = uxTaskGetStackHighWaterMark(NULL);
    osDelay(1);
  }
}

static UBaseType_t remain_HEROShootingTask;
__attribute__((used)) void HEROShootingTask(void *argument)
{
  for(;;)
  {
		static uint16_t timecount1 = 0;
		
		if(g_motor_run_enable == 0)
		{
			//电机急刹车
			if(timecount1 < 150)
			{
				//DJI_MOTOR_EmergencySTOP_ALL(DJI_MFeedback_CAN1,&hcan1,10.0f);
				timecount1++;
			}
		}
		else
		{
			ShootingVel_Calc(5,&SV,RC_Ctl.Switch.S2_R);
			Motor_DJI_ShootingFri(DJI_MFeedback_CAN1,&hcan1,1.0f,SV.UP_Lrpm,SV.UP_Rrpm,SV.Dowm_Mrpm);	
		}
		
//=============================检测剩余栈=================================//
		remain_HEROShootingTask = uxTaskGetStackHighWaterMark(NULL);
    osDelay(1);
  }
}

static UBaseType_t remain_HERODialTask;
__attribute__((used)) void HERODialTask(void *argument)
{

  for(;;)
  {

//=============================检测剩余栈=================================//
		remain_HERODialTask = uxTaskGetStackHighWaterMark(NULL);
    osDelay(1);
  }
}

void get_chassis_data(void)
{ 
	 Chassis_Move.RelativeAngle_Radian = DJI_MFeedback_CAN1[5].angle_raw;
	 Chassis_Move.RelativeAngle_Radian = DJI_MFeedback_CAN1[5].angle_deg;
	
	 Chassis_Move.Vel.FB = MyMath_Map_Range(RC_Ctl.Stick.LX,-HOTRC_RANGE,HOTRC_RANGE,-100.0f,100.0f);
	 Chassis_Move.Vel.RL = MyMath_Map_Range(RC_Ctl.Stick.LY,-HOTRC_RANGE,HOTRC_RANGE,-100.0f,100.0f);	
	
	 Chassis_Mec.FL.C_rpm  = DJI_MFeedback_CAN1[0].speed_rpm;
	 Chassis_Mec.FL.C_Curr = DJI_MFeedback_CAN1[0].current_ma;
	
	 Chassis_Mec.FR.C_rpm  = DJI_MFeedback_CAN1[1].speed_rpm;
	 Chassis_Mec.FR.C_Curr = DJI_MFeedback_CAN1[1].current_ma;
	
	 Chassis_Mec.BL.C_rpm  = DJI_MFeedback_CAN1[2].speed_rpm;
	 Chassis_Mec.BL.C_Curr = DJI_MFeedback_CAN1[2].current_ma;
	
	 Chassis_Mec.BR.C_rpm  = DJI_MFeedback_CAN1[2].speed_rpm;
	 Chassis_Mec.BR.C_Curr = DJI_MFeedback_CAN1[2].current_ma;
}

void get_gimbal_data(void)
{ 	

}