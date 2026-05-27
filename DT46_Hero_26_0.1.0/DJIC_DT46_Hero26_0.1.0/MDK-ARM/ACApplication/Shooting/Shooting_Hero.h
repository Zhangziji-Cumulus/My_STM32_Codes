#ifndef SHOOTING_HERO_H_
#define SHOOTING_HERO_H_

#include "A_MCommon.h"

//** #################################################################################################### **//
//** ====================================== 定义数据、结构体 ============================================= **//
//** #################################################################################################### **//

#if((BOARD_MODE == BOARD_MODE_DUAL))

//** #################################################################################################### **//
//** ====================================== 定义数据、结构体 ============================================= **//
//** #################################################################################################### **//

/* 
    自动装弹思路，摩擦轮检测电流，电流突变很大一段时间，判断发出一颗弹丸，给一个标志；
    推杆向后给出空间后，给一个标志；拨盘上弹，检测电流过大后，给标志。
*/

// 零点设置状态枚举
typedef enum {

    ZERO_IDLE           = 0,    // 空闲状态，未开始找零点
    ZERO_FINDING        = 1,    // 正在寻找零点（运动/检测中）
    ZERO_SUCCESS        = 2,    // 零点设置成功
    ZERO_FAIL           = 3,    // 零点设置失败
    ZERO_COMPLETE       = 4     // 流程完成（可选）

} ZDT_Set_Zero_State_e;

#if(ROBOT_TYPE == ROBOTTYPE_HERO)

typedef struct 
{
    //接收控制指令结构体
    CMD_t CMD;
    //陀螺仪数据
    const fp32* INS_angle;

    //电机反馈数据
    struct{
        const DJI_MotorFeedback_t* Ptr; //反馈电机数据

        DJI_MotorFeedback_t UL;
        DJI_MotorFeedback_t UR;
        DJI_MotorFeedback_t DM;

        DJI_MotorFeedback_t DIAL;
    }DJI_Motordata;

    struct{

        //摩擦轮
        struct{
            
            float ShootingSpeed;

            struct
            {
	            int16_t T_rpm;     //目标转速
                 int16_t Ctrl_Vel;  //最终控制值，(PID计算输出值)
            }UL;

            struct
            {
	            int16_t T_rpm;      //目标转速
                 int16_t Ctrl_Vel;  //最终控制值，(PID计算输出值)
            }UR;

            struct
            {
	            int16_t T_rpm;     //目标转速
                int16_t Ctrl_Vel;  //最终控制值，(PID计算输出值)
            }DM;

        }Friction;

        //拨盘
        struct{

            float   T_Angle;   //目标角度
            float   T_Speed;   //目标速度
	        int16_t T_rpm;     //目标转速
            int16_t Ctrl_Vel;  //最终控制值，(PID计算输出值)

        }Dial;

        //推杆
        struct
        {
            ZDT_Set_Zero_State_e ZeroState;  // 零点状态机
            float   T_Angle;
            int16_t Ctrl_Vel;
        }PushRod;
        

    }Calc;

}Shooting_Instance_t;

#endif

#endif

#endif // SHOOTING_HERO_H_