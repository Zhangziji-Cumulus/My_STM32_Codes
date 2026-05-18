#ifndef ROBOT_HERO_CONFIG_H_
#define ROBOT_HERO_CONFIG_H_

#include "APP_Config.h"

//** ------------------------------------------------------------ **//
//** ====================== Auto Config ========================= **//
//** ------------------------------------------------------------ **//


#define AUTO_AIM                AutoAim_OFF         //自动瞄准配置
#define AUTO_SHOOTING           AutoShooting_OFF    //自动射击配置
#define AUTO_NAVIGATION         AutoNavigation_OFF  //自动导航配置

//** ------------------------------------------------------------ **//
//** ===================== Gimbal Config ======================== **//
//** ------------------------------------------------------------ **//

#define GIMBAL_MOTOR_TYPE_YAW    DJI_GM6020         //云台电机类型
#define GIMBAL_MOTOR_TYPE_PITCH  DJI_M3508          //云台电机类型

//** ------------------------------------------------------------ **//
//** ==================== Shooting Config ======================= **//
//** ------------------------------------------------------------ **//

/* 摩擦轮配置 */

#define FRICTION_NUM            3                   //摩擦轮数量
#define FRICTION_RADIUS         30                  //摩擦轮半径（单位：mm）
#define FRICTION_MAX_SPEED      2                   //摩擦轮最大线速度

#define FRICTION_CAN            hcan1               //摩擦轮CAN总线

#define FRICTION_MOTOR_TYPE     DJI_M3508           //摩擦轮电机类型

#define FRICTION_MOTOR_ID_UL    1                   //上左摩擦轮电机ID
#define FRICTION_MOTOR_ID_UR    2                   //上右摩擦轮电机ID
#define FRICTION_MOTOR_ID_DM    3                   //下中摩擦轮电机ID

/* 拨盘配置 */

#define DIAL_CAN                hcan1               //拨盘CAN总线

#define DIAL_MOTOR_TYPE         DJI_M3508           //拨盘电机类型
#define DIAL_MOTOR_ID           4                   //拨盘电机ID

//** ------------------------------------------------------------ **//
//** ==================== Chassis Config ======================== **//
//** ------------------------------------------------------------ **//

#define CHASSIS_TYPE            Mecanum             //底盘类型
#define CHASSIS_MOTOR_TYPE      DJI_M3508           //底盘电机类型

//从前左开始，顺时针数电机ID（需与实际连接的电机ID对应）
#define CHASSIS_MOTOR_ID_FL      3                   //前左底盘电机ID
#define CHASSIS_MOTOR_ID_FR      2                   //前右底盘电机ID
#define CHASSIS_MOTOR_ID_BL      0                   //后左底盘电机ID
#define CHASSIS_MOTOR_ID_BR      1                   //后右底盘电机ID



//** ------------------------------------------------------------ **//
//** ===================== Control PID ========================== **//
//** ------------------------------------------------------------ **//

#endif // ROBOT_HERO_CONFIG_H_