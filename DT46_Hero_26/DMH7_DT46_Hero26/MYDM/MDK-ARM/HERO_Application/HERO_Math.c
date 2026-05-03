#include "HERO_Math.h"

//输入值达到32767时会溢出，从32767到-32767
float RPM_Protect(float rpm)
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

/**

@brief 计算轮子直线运动速度

@param radius 轮子半径，单位：m

@param rpm 轮子转速，单位：转/分钟

@return 线速度，单位：m/s */
float calc_wheel_speed(float radius, float rpm) {
	
	// 周长
	float circumference = 2 * MY_PI * radius; 
	// 每分钟行驶距离 
	float dist_per_min = circumference * rpm; 
	// 转换为每秒速度 
	float speed = dist_per_min / 60.0f;

return speed;

}

/**
 * @brief 电机转速(rpm) → 实际线速度(m/s)
 * @param radius_m      轮子半径，单位：m
 * @param motor_rpm     电机实际转速（来自编码器），单位：rpm
 * @param gear_ratio    减速比
 * @return              线速度，单位：m/s
 */
float calc_wheel_rpm(float speed, float radius)
{
	float rpm = (speed * 60.0f) / (2 * MY_PI * radius);

	rpm = RPM_Protect(rpm);
	 
	return rpm;
}

/**
 * @brief 线速度(m/s) → 电机目标转速(rpm)
 * @param speed_mps     目标线速度，单位：m/s
 * @param radius_m      轮子半径，单位：m
 * @param gear_ratio    减速比（电机转速/轮子转速），如 10:1 则填 10.0
 * @return              电机目标转速，单位：rpm
 */
float calc_motor_rpm_from_speed(float speed_mps, float radius_m, float gear_ratio)
{
    if (radius_m <= 0.0f || gear_ratio <= 0.0f) return 0.0f;
    
    // 轮子转速 × 减速比 = 电机转速
    float wheel_rpm = (speed_mps * 60.0f) / (2.0f * MY_PI * radius_m);
    float motor_rpm = wheel_rpm * gear_ratio;
    
    return RPM_Protect(motor_rpm); // ?? 限幅必须基于电机最大允许转速
}

/**
 * @brief 电机转速(rpm) → 实际线速度(m/s)
 * @param radius_m      轮子半径，单位：m
 * @param motor_rpm     电机实际转速（来自编码器），单位：rpm
 * @param gear_ratio    减速比
 * @return              线速度，单位：m/s
 */
float calc_speed_from_motor_rpm(float radius_m, float motor_rpm, float gear_ratio)
{
    if (radius_m <= 0.0f || gear_ratio <= 0.0f) return 0.0f;
    
    float wheel_rpm = motor_rpm / gear_ratio;
    float circumference = 2.0f * MY_PI * radius_m;
    
    return (circumference * wheel_rpm) / 60.0f;
}
