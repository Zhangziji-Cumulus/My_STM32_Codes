#ifndef __HERO_MATH_H
#define __HERO_MATH_H

#define MY_PI 3.14159265358979323846

//输入值达到32767时会溢出，从32767到-32767
float RPM_Protect(float rpm);
/**
* @brief 计算轮子直线运动速度
* @param radius 轮子半径，单位：m
* @param rpm 轮子转速，单位：转/分钟
* @return 线速度，单位：m/s */
float calc_wheel_speed(float radius, float rpm);
/**
 * @brief 电机转速(rpm) → 实际线速度(m/s)
 * @param radius_m      轮子半径，单位：m
 * @param motor_rpm     电机实际转速（来自编码器），单位：rpm
 * @param gear_ratio    减速比
 * @return              线速度，单位：m/s
 */
float calc_wheel_rpm(float speed, float radius);
/**
 * @brief 线速度(m/s) → 电机目标转速(rpm)
 * @param speed_mps     目标线速度，单位：m/s
 * @param radius_m      轮子半径，单位：m
 * @param gear_ratio    减速比（电机转速/轮子转速），如 10:1 则填 10.0
 * @return              电机目标转速，单位：rpm
 */
float calc_motor_rpm_from_speed(float speed_mps, float radius_m, float gear_ratio);
/**
 * @brief 电机转速(rpm) → 实际线速度(m/s)
 * @param radius_m      轮子半径，单位：m
 * @param motor_rpm     电机实际转速（来自编码器），单位：rpm
 * @param gear_ratio    减速比
 * @return              线速度，单位：m/s
 */
float calc_speed_from_motor_rpm(float radius_m, float motor_rpm, float gear_ratio);


#endif
