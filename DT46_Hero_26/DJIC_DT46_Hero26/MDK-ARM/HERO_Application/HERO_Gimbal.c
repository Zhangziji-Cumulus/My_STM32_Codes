#include "HERO_Gimbal.h"

float tesppt;

void HERO_Gimbal_YawStable(Gimbal_Data_t* GD,int16_t RC_ctl)
{
	tesppt = MyMath_Map_Range(RC_ctl,-HOTRC_RANGE,HOTRC_RANGE,-0.1f,0.1f);
	GD->TAngle.Yaw = GD->TAngle.Yaw + tesppt;
	GD->TAngle.Yaw = MyMath_Limit_Float(GD->TAngle.Yaw,-180.00f,180.00f,1);

	GD->CAngle.Yaw = MyMath_Limit_Float(GD->IMU.Yaw,-180.00f,180.00f,1);
}

void HERO_Gimbal_PitchStable(void)
{
	
}