#include "Shooting_Hero.h"

#if((BOARD_MODE == BOARD_MODE_DUAL && BOARD_ID == GIMBAL_BOARD ))

#if(ROBOT_TYPE == ROBOTTYPE_HERO)

//** #################################################################################################### **//
//** ====================================== 定义数据、结构体 ============================================= **//
//** #################################################################################################### **//

PID_HandleTypeDef SFri_UL_In;
PID_HandleTypeDef SFri_UL_Ex;

PID_HandleTypeDef SFri_UR_In;
PID_HandleTypeDef SFri_UR_Ex;

PID_HandleTypeDef SFri_DM_In;
PID_HandleTypeDef SFri_DM_Ex;

//** #################################################################################################### **//
//** ========================================= 对内函数声明 ============================================== **//
//** #################################################################################################### **//



//** #################################################################################################### **//
//** ====================================== 对外若定义覆盖函数 =========================================== **//
//** #################################################################################################### **//

//初始化函数
void Shooting_Init(void)
{
	//摩擦轮PID
	PID_Init(&SFri_UL_In,1.0f,0.0f,0.0f,-DJI_M3508_R,DJI_M3508_R,-10.0f, 10.0f);
	PID_Init(&SFri_UL_Ex,30.0f,0.0f,0.0f,-10000,10000,-10.0f, 10.0f);
	
	PID_Init(&SFri_UR_In,1.0f,0.0f,0.0f,-DJI_M3508_R,DJI_M3508_R,-10.0f, 10.0f);
	PID_Init(&SFri_UR_Ex,30.0f,0.0f,0.0f,-10000,10000,-10.0f, 10.0f);
	
	PID_Init(&SFri_DM_In,1.0f,0.0f,0.0f,-DJI_M3508_R,DJI_M3508_R,-10.0f, 10.0f);
	PID_Init(&SFri_DM_Ex,30.0f,0.0f,0.0f,-10000,10000,-10.0f, 10.0f);
	
}          

//更新状态函数
void Shooting_Update(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}         

//异常处理函数
void Shooting_HandleError(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

//设置模式
void Shooting_SetMode(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

//更新目标量
void Shooting_RefreshTarget(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

//计算控制量
void Shooting_CtrlCalc(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

//发送控制指令
void Shooting_SendCmd(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

#endif

#endif

#if((BOARD_MODE == BOARD_MODE_DUAL && BOARD_ID == CHASSIS_BOARD ))

#if(ROBOT_TYPE == ROBOTTYPE_HERO)

//** #################################################################################################### **//
//** ====================================== 定义数据、结构体 ============================================= **//
//** #################################################################################################### **//

static Shooting_Instance_t Shooting_Instance;

PID_HandleTypeDef Dial_Motor_STOP;

PID_HandleTypeDef Dial_In;
PID_HandleTypeDef Dial_Ex;

//** #################################################################################################### **//
//** ========================================= 对内函数声明 ============================================== **//
//** #################################################################################################### **//

static void Shooting_Update_Target(void);

//** #################################################################################################### **//
//** ====================================== 对外若定义覆盖函数 =========================================== **//
//** #################################################################################################### **//

//初始化函数
void Shooting_Init(void)
{
    //初始模式STOP
    Shooting_Instance.CMD.ctrl = STOP_MODE;

    PID_Init(&Dial_Motor_STOP,3.0f,0.0f,0.0f,-DJI_M3508_R,DJI_M3508_R,-5.0f, 5.0f);

	//拨盘PID
	PID_Init(&Dial_In,1.0f,0.0f,0.0f,-2500,2500,-10.0f, 10.0f);
	PID_Init(&Dial_Ex,30.0f,0.0f,0.0f,-10000,10000,-10.0f, 10.0f);

}          

//更新状态函数
void Shooting_Update(void)
{
    //获取控制命令数据结构体指针
    Shooting_Instance.CMD = *CMD_Get_point();

    //获取电机反馈数据指针
    Shooting_Instance.DJI_Motordata.Ptr = MotorCtrl_DJI_GetDJI_MFeedback(&DIAL_CAN_CTRL);
    Shooting_Instance.DJI_Motordata.DIAL = Shooting_Instance.DJI_Motordata.Ptr[DIAL_MOTOR_ID_FBK];

}         

//异常处理函数
void Shooting_HandleError(void)
{

}

//设置模式
void Shooting_SetMode(void)
{

}

//更新目标量
void Shooting_RefreshTarget(void)
{
    Shooting_Update_Target();
}

//计算控制量
void Shooting_CtrlCalc(void)
{
    Shooting_Instance.Calc.Dial.Ctrl_Vel = PID_Double_Calculate(&Dial_In,
                                                                &Dial_Ex,
                                                                Shooting_Instance.Calc.Dial.T_rpm,
                                                                Shooting_Instance.DJI_Motordata.DIAL.current_ma,
                                                                Shooting_Instance.DJI_Motordata.DIAL.speed_rpm,
                                                                DIAL_PID_THRESHOLD);

}

//发送控制指令
void Shooting_SendCmd(void)
{
        // 静态变量：显式初始化，记录停止状态与起始时间
    static uint32_t stop_start_time = 0;
    static bool is_stopping = false;

    // 获取当前系统时间(ms)
    uint32_t now_time = HAL_GetTick();

    if(Shooting_Instance.CMD.ctrl == STOP_MODE)
    {
        
        if (!is_stopping)
        {
            stop_start_time = now_time;
            is_stopping = true;
        }
        // 无溢出安全判断：急停周期内执行紧急停止
        if ((now_time - stop_start_time) < DJI_MOTOR_STOP_TIME_MS)
        {
            int16_t PIDSTOP = { 0 };

			PIDSTOP = PID_Calculate(&Dial_Motor_STOP,Shooting_Instance.DJI_Motordata.DIAL.speed_rpm,0);
           
			ESC_Control_Raw_Single(&DIAL_CAN_CTRL,DIAL_CAN_ID,PIDSTOP);
		}
        else
        {
            ESC_Control_Raw_Single(&DIAL_CAN_CTRL,DIAL_CAN_ID,0);
        }

    }
    else
    {
        // 退出停止模式，重置状态
        is_stopping = false;

        ESC_Control_Raw_Single(&DIAL_CAN_CTRL,DIAL_CAN_ID,Shooting_Instance.Calc.Dial.Ctrl_Vel);
    }

}

//** #################################################################################################### **//
//** ========================================= 对内算法函数 ============================================== **//
//** #################################################################################################### **//

static void Shooting_Update_Target(void)
{
    if(Shooting_Instance.CMD.Shooting.Load == ON)
    {
        Shooting_Instance.Calc.Dial.T_Speed = DIAL_MAX_SPEED_M_S;
        Shooting_Instance.Calc.Dial.T_rpm = (int16_t)calc_motor_rpm_from_speed(Shooting_Instance.Calc.Dial.T_Speed,(DIAL_RADIUS_MM / 1000),DIAL_RATIO);

    }
    else
    {
        Shooting_Instance.Calc.Dial.T_rpm = 0;
    }
}

#endif

#endif