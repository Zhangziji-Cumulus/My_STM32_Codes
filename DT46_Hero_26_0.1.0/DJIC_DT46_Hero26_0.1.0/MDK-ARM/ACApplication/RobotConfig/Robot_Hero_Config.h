#ifndef ROBOT_HERO_CONFIG_H_
#define ROBOT_HERO_CONFIG_H_

#include "APP_Config.h"

#if(ROBOT_TYPE == ROBOTTYPE_HERO)

//** ------------------------------------------------------------ **//
//** ================== Hero System Config ====================== **//
//** ------------------------------------------------------------ **//



//** ------------------------------------------------------------ **//
//** ===================== Control PID ========================== **//
//** ------------------------------------------------------------ **//

#define MECANUM_PID_MERROR  1.0f



//** ------------------------------------------------------------ **//
//** ====================== Auto Config ========================= **//
//** ------------------------------------------------------------ **//

#define AUTO_AIM                AUTO_AIM_OFF         //自动瞄准配置
#define AUTO_SHOOTING           AUTO_SHOOTING_OFF    //自动射击配置
#define AUTO_NAVIGATION         AUTO_NAVIGATION_OFF  //自动导航配置

//** ------------------------------------------------------------ **//
//** ===================== Gimbal Config ======================== **//
//** ------------------------------------------------------------ **//

/* 云台速度相关 */

/* 云台物理参数 */
#define GIMBAL_YAW_RATIO    2       //Yaw轴减速比为 2:1

/* 云台电机控制 */
#define GIMBAL_CAN_CTRL     hcan2   //云台CAN总线

#define GIMBAL_YAW_MOTOR_TYPE    MOTOR_DJI_GM6020         //云台电机类型
#define GIMBAL_PITCH_MOTOR_TYPE  MOTOR_DJI_M3508          //云台电机类型

#define GIMBAL_MOTOR_ID_CAN_YAW      4
#define GIMBAL_MOTOR_ID_CAN_PITCH    0

//** ------------------------------------------------------------ **//
//** ==================== Shooting Config ======================= **//
//** ------------------------------------------------------------ **//

/** ===== 摩擦轮配置 ===== **/

/* 云台速度相关 */
#define FRICTION_MAX_SPEED_M_S  12.0f               //摩擦轮最大线速度(单位：m/s）

/* 摩擦轮的物理参数 */
#define FRICTION_NUM            3                   //摩擦轮数量
#define FRICTION_RADIUS_MM      30                  //摩擦轮半径（单位：mm）

/* 摩擦轮电机控制 */
#define FRICTION_CAN            hcan1               //摩擦轮CAN总线

#define FRICTION_MOTOR_TYPE     MOTOR_DJI_M3508     //摩擦轮电机类型

#define FRICTION_MOTOR_ID_CAN_UL    1                   //上左摩擦轮电机ID
#define FRICTION_MOTOR_ID_CAN_UR    2                   //上右摩擦轮电机ID
#define FRICTION_MOTOR_ID_CAN_DM    3                   //下中摩擦轮电机ID

/** ===== 拨盘配置 ===== **/

/* 拨盘速度相关 */
#define DIAL_MAX_SPEED_M_S      2                   //拨盘最大线速度

/* 拨盘物理参数 */

#define DIAL_RADIUS_MM          0                   //拨盘半径

/* 拨盘电机控制 */
#define DIAL_CAN                hcan1               //拨盘CAN总线

#define DIAL_MOTOR_TYPE         MOTOR_DJI_M3508     //拨盘电机类型
#define DIAL_MOTOR_ID           4                   //拨盘电机ID

//** ------------------------------------------------------------ **//
//** ==================== Chassis Config ======================== **//
//** ------------------------------------------------------------ **//

/* 系统配置 */
#define CHASSIS_TASK_TIME_MS        1               //地盘任务循环时间

/* 地盘类型 */
#define CHASSIS_TYPE        CHASSIS_MECANUM         //底盘类型

/* 地盘速度相关 */
#define CHASSIS_MAX_SPEED            3.0f           //底盘最大移动速度（单位：m/s）
#define CHASSIS_MAX_SPEED_FOLLOWING  2.0f           //底盘最大跟随速度（单位：m/s）
#define CHASSIS_MAX_SPIN_SPEED       8.0f           //底盘最大旋转速度（单位：rad/s）

/* 地盘的物理参数 */
#define CHASSIS_WHEEL_RADIUS_MM     76.0f           //轮子半径（单位：mm）
#define CHASSIS_WHEEL_WB_MM         0.0f            //地盘轴距（单位：mm），前后两轮中心直线距离
#define CHASSIS_WHEEL_TW_MM         0.0f            //地盘轮距（单位：mm），左右车轮中心距离

/* 地盘电机控制 */
#define CHASSIS_CAN_CTRL        hcan1               //底盘CAN控制总线 
#define CHASSIS_CAN_GROUP       1                   //CAN总线控制组，组1
#define CHASSIS_CAN_YAW         hcan1               //获取地盘云台相关角度电机所在总线

#define CHASSIS_MOTOR_TYPE      MOTOR_DJI_M3508     //底盘电机类型

//note: 从前左开始，顺时针数电机ID（需与实际连接的电机ID对应）
#define CHASSIS_MOTOR_ID_CAN_FL     3                   //前左底盘电机ID
#define CHASSIS_MOTOR_ID_CAN_FR     2                   //前右底盘电机ID
#define CHASSIS_MOTOR_ID_CAN_BL     0                   //后左底盘电机ID
#define CHASSIS_MOTOR_ID_CAN_BR     1                   //后右底盘电机ID

/* */
#define YAW_ZERO_ANGLE 271.5f   //云台朝前零点角度重置

#endif

#endif // ROBOT_HERO_CONFIG_H_
