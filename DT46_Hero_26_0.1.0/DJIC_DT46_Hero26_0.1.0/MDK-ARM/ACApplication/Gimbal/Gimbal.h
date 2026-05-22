#ifndef GIMBAL_H_
#define GIMBAL_H_

#include "A_MCommon.h"

#if((BOARD_MODE == BOARD_MODE_DUAL && BOARD_ID == GIMBAL_BOAD )|| BOARD_MODE == BOARD_MODE_SINGLE)

#if(ROBOT_TYPE == ROBOTTYPE_HERO)

#ifndef MAP_CMD_RANGE_TO_M_S_YAW
#define MAP_CMD_RANGE_TO_M_S_YAW(cmd) \
    MyMath_Map_Range_Float((float)(cmd), -CMD_CTRL_RANGE, CMD_CTRL_RANGE, \
                            -CHASSIS_MAX_SPEED, CHASSIS_MAX_SPEED)
#endif

#ifndef MAP_CMD_RANGE_TO_M_S_PITCH
#define MAP_CMD_RANGE_TO_M_S_PITCH(cmd) \
    MyMath_Map_Range_Float((float)(cmd), -CMD_CTRL_RANGE, CMD_CTRL_RANGE, \
                            -CHASSIS_MAX_SPEED, CHASSIS_MAX_SPEED)
#endif

typedef struct 
{
    //接收控制指令结构体
    CMD_t CMD;
    //陀螺仪数据
    const fp32* INS_angle;

    //电机反馈数据
    struct{
        const DJI_MotorFeedback_t* Ptr; //反馈电机数据
        DJI_MotorFeedback_t Yaw;
        DJI_MotorFeedback_t Pitch;
    }MotorData;

    struct{

        //Yaw轴控制量
        struct{

            float   T_Angle;   //目标角度
            float   C_Angle;   //IMU Yaw角度
            int16_t T_Speed;   //目标速度
	        int16_t T_rpm;     //目标转速
            int16_t Ctrl_Vel;  //最终控制值，(PID计算输出值)

        }Yaw;

        //Pitch轴控制量
        struct{

            float   T_Angle;   //目标角度
            float   C_Angle;   //IMU Pitch角度
            int16_t T_Speed;   //目标速度
	        int16_t T_rpm;     //目标转速
            int16_t Ctrl_Vel;  //最终控制值，(PID计算输出值)

        }Pitch;

    }Calc;

}Gimbal_Instance_t;

#endif

#endif

#endif // GIMBAL_H_