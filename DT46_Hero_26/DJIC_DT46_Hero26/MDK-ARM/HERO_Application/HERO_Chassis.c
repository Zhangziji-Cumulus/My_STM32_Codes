#include "HERO_Chassis.h"

//计算轮子直线运动速度
static float calc_wheel_speed(float radius, float rpm);
//由线速度和轮子半径求转速
static float calc_wheel_rpm(float speed, float radius);
//输入值达到32767时会溢出，从32767到-32767
static float RPM_Protect(float rpm);

//地盘运动解算

/*
* @param state 值为1时是地盘跟随模式，值为2时是小陀螺模式
*/
void Chassis_Move_Calc(Chassis_Move_t* pCM,Mecanum_Data_t* pMD,PID_HandleTypeDef *pid,uint8_t state)
{
	pCM->Theta_Radian = pCM->RelativeAngle_Radian - MyMath_Degrees_To_Radians(0.0f);
	
	pCM->Theta_Degree = pCM->RelativeAngle_Degree - 0.0f;
	
	if(state == 1)
	{
		pMD->Target.Xv = pCM->Vel.FB;
		pMD->Target.Yv = pCM->Vel.RL;
		pMD->Target.Rv = PID_Calculate_CycleAngle(pid,pCM->Theta_Degree,0.00f);
	}
	else if(state == 2)
	{
		pMD->Target.Xv = ((pCM->Vel.RL * cos(pCM->RelativeAngle_Radian)) - (pCM->Vel.FB * sin(pCM->RelativeAngle_Radian)));
		pMD->Target.Yv = ((pCM->Vel.RL * sin(pCM->RelativeAngle_Radian)) + (pCM->Vel.FB * cos(pCM->RelativeAngle_Radian)));
		pMD->Target.Rv = 500;
	}
	
	Chassis_Mecanum_Calc(pMD);
}

//每个麦轮速度解算
void Chassis_Mecanum_Calc(Mecanum_Data_t* pMD)
{
	pMD->FL.T_Speed = -((pMD->Target.Xv) - (pMD->Target.Yv) + (pMD->Target.Rv));
	pMD->FR.T_Speed =  ((pMD->Target.Xv) + (pMD->Target.Yv) - (pMD->Target.Rv));
	pMD->BL.T_Speed =  ((pMD->Target.Xv) - (pMD->Target.Yv) - (pMD->Target.Rv));
	pMD->BR.T_Speed = -((pMD->Target.Xv) + (pMD->Target.Yv) + (pMD->Target.Rv));
	
	pMD->FL.T_rpm = calc_wheel_rpm(pMD->FL.T_Speed,(MECANUM_WHEEL_RADIUS_MM/1000.0f));
	pMD->FR.T_rpm = calc_wheel_rpm(pMD->FR.T_Speed,(MECANUM_WHEEL_RADIUS_MM/1000.0f));
	pMD->BL.T_rpm = calc_wheel_rpm(pMD->BL.T_Speed,(MECANUM_WHEEL_RADIUS_MM/1000.0f));
	pMD->BR.T_rpm = calc_wheel_rpm(pMD->BR.T_Speed,(MECANUM_WHEEL_RADIUS_MM/1000.0f));
}

/**
 * @brief 计算轮子直线运动速度
 * @param radius 轮子半径，单位：m
 * @param rpm    轮子转速，单位：转/分钟
 * @return       线速度，单位：m/s
 */
static float calc_wheel_speed(float radius, float rpm)
{
    // 周长
    float circumference = 2 * MY_PI * radius;
    // 每分钟行驶距离
    float dist_per_min = circumference * rpm;
    // 转换为每秒速度
    float speed = dist_per_min / 60.0f;

    return speed;
}

/**
 * @brief 由线速度和轮子半径求转速
 * @param speed   线速度，单位：m/s
 * @param radius  轮子半径，单位：m
 * @return        转速，单位：转/分钟 (rpm)
 */
static float calc_wheel_rpm(float speed, float radius)
{
    if (radius <= 0.0f)
    {
        return 0.0f; // 防止除0
    }

    float rpm = (speed * 60.0f) / (2 * MY_PI * radius);
		
		rpm = RPM_Protect(rpm);
		
    return rpm;
}

//输入值达到32767时会溢出，从32767到-32767
static float RPM_Protect(float rpm)
{
	if(rpm >= 32767.0f)
	{
		rpm = 32767.0f;
	}
	else if(rpm <= -32767.0f)
	{
		rpm = -32767.0f;
	}
		
	return rpm;
}

