#include "HERO_DriveSystem.h"
#include "HERO_API.h"

extern Dual_Board_Transmit_t DBT_RX;


extern HOTRC_Ctl_t RC_Ctl;
extern DJI_MotorFeedback_t DJI_MFeedback[8];
extern fp32 INS_angle[3];


//定义地盘数据
extern PID_HandleTypeDef Chassis_Follow_PID;
Mecanum_Data_t Chassis_Mec;
Chassis_Move_t Chassis_Move;
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
		
		Chassis_Move_Calc(&Chassis_Move,&Chassis_Mec,&Chassis_Follow_PID,RC_Ctl.Switch.S2_R);
			
		Motor_DJI_SpeedCtl_1_4(&hcan1,1.0f,Chassis_Mec.FL.T_rpm,Chassis_Mec.FL.T_rpm,Chassis_Mec.FL.T_rpm,Chassis_Mec.FL.T_rpm);
		
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
		
		HERO_Gimbal_YawStable(&GD,RC_Ctl.Stick.RX);
		HERO_Gimbal_PitchStable(&GD,RC_Ctl.Stick.RY);
		
		Motor_DJI_IMUYawContral(GD.TAngle.Pitch,MyMath_Radians_To_Degrees(DBT_RX.B2.IMU[2]),6,36);
		Motor_DJI_IMUPitchContral(GD.TAngle.Yaw,MyMath_Radians_To_Degrees(DBT_RX.B2.IMU[0]),5,36);
		//Motor_DJI_Angle_SingleContral(GD.TAngle.Yaw,5,36);
		
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
	 Chassis_Move.RelativeAngle_Radian = DJI_MFeedback[5].angle_raw;
	 Chassis_Move.RelativeAngle_Radian = DJI_MFeedback[5].angle_deg;
	
	 Chassis_Move.Vel.FB = MyMath_Map_Range(RC_Ctl.Stick.LX,-HOTRC_RANGE,HOTRC_RANGE,-100.0f,100.0f);
	 Chassis_Move.Vel.RL = MyMath_Map_Range(RC_Ctl.Stick.LY,-HOTRC_RANGE,HOTRC_RANGE,-100.0f,100.0f);	
	 Chassis_Mec.FL.C_rpm  = DJI_MFeedback[0].speed_rpm;
	 Chassis_Mec.FL.C_Curr = DJI_MFeedback[0].current_ma;
	
}

void get_gimbal_data(void)
{ 	

}