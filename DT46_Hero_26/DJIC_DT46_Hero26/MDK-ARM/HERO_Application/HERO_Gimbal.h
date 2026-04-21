#ifndef __HERO_GIMBAL_H
#define __HERO_GIMBAL_H

#include "main.h"
#include "HERO_API.h"

typedef struct{

	struct{
		float Yaw;
		float Pitch;
	}TAngle;
	
	struct{
		float Yaw;
		float Pitch;
	}IMU;
	
	struct{
		float Yaw;
		float Pitch;
	}CAngle;
	
}Gimbal_Data_t;


void HERO_Gimbal_YawStable(Gimbal_Data_t* GD,int16_t RC_ctl);

#endif
