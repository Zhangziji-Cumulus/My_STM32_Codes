#include "Chassis_Mecanu.h"

#if(CHASSIS_TYPE == CHASSIS_MECANUM)

static Chassis_Instance_t Chassis_Instance;

//初始化函数
void Chassis_Init(void)
{
   Chassis_Instance.CMD.ctrl = STOP_MODE;
}          

//更新状态函数
void Chassis_Update(void)
{
    //获取控制命令数据结构体
    Chassis_Instance.CMD = *CMD_Get_point();

    // 获取IMU数据指针并复制到数组中
    Chassis_Instance.INS_angle = IMU_Get_point();

    //获取电机反馈数据指针
    Chassis_Instance.Motor.MotorFeedback_Ptr = MotorCtrl_DJI_GetDJI_MFeedback(&CHASSIS_CAN_CTRL);
    //更新电机数据
    Chassis_Instance.Motor.W_FL = Chassis_Instance.Motor.MotorFeedback_Ptr[CHASSIS_MOTOR_ID_CAN_FL];  // 左前轮
    Chassis_Instance.Motor.W_FR = Chassis_Instance.Motor.MotorFeedback_Ptr[CHASSIS_MOTOR_ID_CAN_FR];  // 右前轮
    Chassis_Instance.Motor.W_BL = Chassis_Instance.Motor.MotorFeedback_Ptr[CHASSIS_MOTOR_ID_CAN_BL];  // 左后轮
    Chassis_Instance.Motor.W_BR = Chassis_Instance.Motor.MotorFeedback_Ptr[CHASSIS_MOTOR_ID_CAN_BR];  // 右后轮


    //设置地盘控制目标值
    Chassis_Instance.Calc.Target.FB = MAP_CMD_RANGE_TO_M_S(Chassis_Instance.CMD.Chassis.FB);
    Chassis_Instance.Calc.Target.LR = MAP_CMD_RANGE_TO_M_S(Chassis_Instance.CMD.Chassis.LR);
    Chassis_Instance.Calc.Target.RO = MAP_CMD_RANGE_TO_M_S(Chassis_Instance.CMD.Chassis.RO);

    //地盘跟随所需要的差角CHASSIS_CAN_YAW
    Chassis_Instance.Calc.YawAnale_Ptr = MotorCtrl_DJI_GetDJI_MFeedback(&CHASSIS_CAN_YAW);
    Chassis_Instance.Calc.Yaw_Angle = Chassis_Instance.Calc.YawAnale_Ptr[GIMBAL_MOTOR_ID_CAN_YAW].angle_deg;

    float tempAngle = MyMath_normalize_m180_to_p180(Chassis_Instance.Calc.Yaw_Angle - YAW_ZERO_ANGLE);
	//  float testangle = MyMath_normalize_m180_to_p180(DJI_MFeedback_CAN3[4].angle_deg - YAW_ZERO_ANGLE);

	//  Chassis_Move.RelativeAngle_Degree = MyMath_cal_output_angle(testangle,2);//- ERRORANGLE
	//  Chassis_Move.RelativeAngle_Degree = MyMath_normalize_m180_to_p180(Chassis_Move.RelativeAngle_Degree);
	
	//  Chassis_Move.RelativeAngle_Radian = MyMath_Degrees_To_Radians(Chassis_Move.RelativeAngle_Degree);

    //小陀螺的相对角度
}         

//异常处理函数
void Chassis_HandleError(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

//设置模式
void Chassis_SetMode(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

//更新目标量
void Chassis_RefreshTarget(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

//计算控制量
void Chassis_CtrlCalc(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

//发送控制指令
void Chassis_SendCmd(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}      

#endif