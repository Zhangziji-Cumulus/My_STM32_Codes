#include "Shooting_Hero.h"

#if((BOARD_MODE == BOARD_MODE_DUAL && BOARD_ID == GIMBAL_BOARD ))

#if(ROBOT_TYPE == ROBOTTYPE_HERO)

//** #################################################################################################### **//
//** ====================================== 定义数据、结构体 ============================================= **//
//** #################################################################################################### **//

static Shooting_Instance_t Shooting_Instance;

PID_HandleTypeDef PID_SFri_STOP;

PID_HandleTypeDef PID_SFri_UL_In;
PID_HandleTypeDef PID_SFri_UL_Ex;

PID_HandleTypeDef PID_SFri_UR_In;
PID_HandleTypeDef PID_SFri_UR_Ex;

PID_HandleTypeDef PID_SFri_DM_In;
PID_HandleTypeDef PID_SFri_DM_Ex;

//** #################################################################################################### **//
//** ========================================= 对内函数声明 ============================================== **//
//** #################################################################################################### **//

static void Friction_Update_Target(void);
static void PuahRod_Update_Target(void);

//** #################################################################################################### **//
//** ====================================== 对外若定义覆盖函数 =========================================== **//
//** #################################################################################################### **//

//初始化函数
void Shooting_Init(void)
{
    //初始模式STOP
    Shooting_Instance.CMD.ctrl = STOP_MODE;

    //3508电机急停
    PID_Init(&PID_SFri_STOP,3.0f,0.0f,0.0f,-DJI_M3508_R,DJI_M3508_R,-5.0f, 5.0f);

	//摩擦轮PID
	PID_Init(&PID_SFri_UL_In,1.0f,0.0f,0.0f,-DJI_M3508_R,DJI_M3508_R,-10.0f, 10.0f);
	PID_Init(&PID_SFri_UL_Ex,30.0f,0.0f,0.0f,-10000,10000,-10.0f, 10.0f);
	
	PID_Init(&PID_SFri_UR_In,1.0f,0.0f,0.0f,-DJI_M3508_R,DJI_M3508_R,-10.0f, 10.0f);
	PID_Init(&PID_SFri_UR_Ex,30.0f,0.0f,0.0f,-10000,10000,-10.0f, 10.0f);
	
	PID_Init(&PID_SFri_DM_In,1.0f,0.0f,0.0f,-DJI_M3508_R,DJI_M3508_R,-10.0f, 10.0f);
	PID_Init(&PID_SFri_DM_Ex,30.0f,0.0f,0.0f,-10000,10000,-10.0f, 10.0f);
	
    Shooting_Instance.Calc.PushRod.State = PUSH_FRONT_ING;

	//X_V2_Auto_Return_Sys_Params_Timed(1,S_CPHA,15);//读取张大头步进电机的实际工作电流
}

//更新状态函数
void Shooting_Update(void)
{
    // static uint8_t ZDT_Time_Count = 0;

    // if(ZDT_Time_Count < 10 / SHOOTING_TASK_TIME_MS)
    // {
    //     X_V2_Read_System_State_Params(1);
    //     ZDT_Time_Count = 0;
    // }
    // ZDT_Time_Count++;

    Shooting_Instance.CMD = *CMD_Get_point();
    Shooting_Instance.DJI_Motordata.Ptr = MotorCtrl_DJI_GetDJI_MFeedback(&FRICTION_CAN_CTRL);
    Shooting_Instance.DJI_Motordata.UL = Shooting_Instance.DJI_Motordata.Ptr[FRICTION_MOTOR_ID_FBK_UL];
    Shooting_Instance.DJI_Motordata.UR = Shooting_Instance.DJI_Motordata.Ptr[FRICTION_MOTOR_ID_FBK_UR];
    Shooting_Instance.DJI_Motordata.DM = Shooting_Instance.DJI_Motordata.Ptr[FRICTION_MOTOR_ID_FBK_DM];
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
    Friction_Update_Target();
    PuahRod_Update_Target();
}

//计算控制量
void Shooting_CtrlCalc(void)
{
    Shooting_Instance.Calc.Friction.UL.Ctrl_Vel = 
                                    PID_Double_Calculate(&PID_SFri_UL_In,
                                                         &PID_SFri_UL_Ex,
                                                         Shooting_Instance.Calc.Friction.UL.T_rpm,
                                                         Shooting_Instance.DJI_Motordata.UL.current_ma,
                                                         Shooting_Instance.DJI_Motordata.UL.speed_rpm,
                                                         FRICTION_PID_THRESHOLD);

    Shooting_Instance.Calc.Friction.UR.Ctrl_Vel = 
                                    PID_Double_Calculate(&PID_SFri_UR_In,
                                                         &PID_SFri_UR_Ex,
                                                         Shooting_Instance.Calc.Friction.UR.T_rpm,
                                                         Shooting_Instance.DJI_Motordata.UR.current_ma,
                                                         Shooting_Instance.DJI_Motordata.UR.speed_rpm,
                                                         FRICTION_PID_THRESHOLD);

    Shooting_Instance.Calc.Friction.DM.Ctrl_Vel = 
                                    PID_Double_Calculate(&PID_SFri_DM_In,
                                                         &PID_SFri_DM_Ex,
                                                         Shooting_Instance.Calc.Friction.DM.T_rpm,
                                                         Shooting_Instance.DJI_Motordata.DM.current_ma,
                                                         Shooting_Instance.DJI_Motordata.DM.speed_rpm,
                                                         FRICTION_PID_THRESHOLD);
}

//发送控制指令
void Shooting_SendCmd(void)
{
    //ZDT使能标志
    static uint8_t ZDT_Enable_Sate = 0;

    // 静态变量：显式初始化，记录停止状态与起始时间
    static uint32_t stop_start_time = 0;
    static bool is_stopping = false;

    // 获取当前系统时间(ms)
    uint32_t now_time = HAL_GetTick();

    if(Shooting_Instance.CMD.ctrl == STOP_MODE)
    {
        int16_t PIDSTOP[4] = { 0 };
				
        if (!is_stopping)
        {
            stop_start_time = now_time;
            is_stopping = true;
        }
        // 无溢出安全判断：急停周期内执行紧急停止
        if ((now_time - stop_start_time) < DJI_MOTOR_STOP_TIME_MS)
        {
			PIDSTOP[FRICTION_MOTOR_ID_FBK_UL] = PID_Calculate(&PID_SFri_STOP,Shooting_Instance.DJI_Motordata.UL.speed_rpm,0);
            PIDSTOP[FRICTION_MOTOR_ID_FBK_UR] = PID_Calculate(&PID_SFri_STOP,Shooting_Instance.DJI_Motordata.UR.speed_rpm,0);
            PIDSTOP[FRICTION_MOTOR_ID_FBK_DM] = PID_Calculate(&PID_SFri_STOP,Shooting_Instance.DJI_Motordata.DM.speed_rpm,0);
           	
            ESC_Control_Raw_Group(&FRICTION_CAN_CTRL,FRICTION_CAN_GROUP,PIDSTOP);
		}
        else
        {
            // 急停超时后：发送零速度指令保持电机锁定（修复失控BUG）
            PIDSTOP[0] = 0;	
			PIDSTOP[1] = 0;
			PIDSTOP[2] = 0;
			PIDSTOP[3] = 0;
			ESC_Control_Raw_Group(&FRICTION_CAN_CTRL,FRICTION_CAN_GROUP,PIDSTOP);
        }

        if(ZDT_Enable_Sate <= 10)
        {
            //失能张大头电机
            X_V2_En_Control(1,false,false);
            ZDT_Enable_Sate++;
        }

    }
    else
    {
        // 退出停止模式，重置状态
        is_stopping = false;

        if(ZDT_Enable_Sate >= 10)
        {
            X_V2_En_Control(1,true,false);
						ZDT_Enable_Sate = 0;
        }

        int16_t PIDoutput[4] = {0};

        PIDoutput[FRICTION_MOTOR_ID_FBK_UL] = Shooting_Instance.Calc.Friction.UL.Ctrl_Vel;
        PIDoutput[FRICTION_MOTOR_ID_FBK_UR] = Shooting_Instance.Calc.Friction.UR.Ctrl_Vel;
        PIDoutput[FRICTION_MOTOR_ID_FBK_DM] = Shooting_Instance.Calc.Friction.DM.Ctrl_Vel;

        ESC_Control_Raw_Group(&FRICTION_CAN_CTRL, FRICTION_CAN_GROUP, PIDoutput);


        if(Shooting_Instance.Calc.PushRod.State = PUSH_BACK_ENTER)
        {
            X_V2_Traj_Pos_LC_Control(PUSHROD_CAN_ID,
                                    PUSHROD_CCW,
                                    PUSHROD_ACC,
                                    PUSHROD_DEC,
                                    PUSHROD_MAX_SPEED_RPM,
                                    Shooting_Instance.Calc.PushRod.T_Angle,
                                    PUSHROD_POS_MODE_ABSOLUTE,
                                    false,
                                    PUSHROD_CURRENT_MAX);
            Shooting_Instance.Calc.PushRod.State = PUSH_BACK_ING;
        }
        else if(Shooting_Instance.Calc.PushRod.State = PUSH_FRONT_ENTER)
        {
            X_V2_Traj_Pos_LC_Control(PUSHROD_CAN_ID,
                                    PUSHROD_CCW,
                                    PUSHROD_ACC,
                                    PUSHROD_DEC,
                                    PUSHROD_MAX_SPEED_RPM,
                                    Shooting_Instance.Calc.PushRod.T_Angle,
                                    PUSHROD_POS_MODE_ABSOLUTE,
                                    false,
                                    PUSHROD_CURRENT_MAX);                 
            Shooting_Instance.Calc.PushRod.State = PUSH_FRONT_ING;
        }
        
    }
}

//** #################################################################################################### **//
//** ========================================= 对内算法函数 ============================================== **//
//** #################################################################################################### **//

static void Friction_Update_Target(void)
{
    if(Shooting_Instance.CMD.Shooting.Friction == ON)
    {

        Shooting_Instance.Calc.Friction.ShootingSpeed = -FRICTION_MAX_SPEED_M_S;

        Shooting_Instance.Calc.Friction.UL.T_rpm = (int16_t)calc_motor_rpm_from_speed(Shooting_Instance.Calc.Friction.ShootingSpeed,(FRICTION_RADIUS_MM / 1000),FRICTION_RATIO);
        Shooting_Instance.Calc.Friction.UR.T_rpm = (int16_t)calc_motor_rpm_from_speed(Shooting_Instance.Calc.Friction.ShootingSpeed,(FRICTION_RADIUS_MM / 1000),FRICTION_RATIO);
        Shooting_Instance.Calc.Friction.DM.T_rpm = (int16_t)calc_motor_rpm_from_speed(Shooting_Instance.Calc.Friction.ShootingSpeed,(FRICTION_RADIUS_MM / 1000),FRICTION_RATIO);
        
    }
    else
    {
        Shooting_Instance.Calc.Friction.UL.T_rpm = 0;
        Shooting_Instance.Calc.Friction.UR.T_rpm = 0;
        Shooting_Instance.Calc.Friction.DM.T_rpm = 0;
    }
}

static void PuahRod_Update_Target(void)
{

    if((Shooting_Instance.CMD.Shooting.Fire == ON) && (Shooting_Instance.Calc.PushRod.State = PUSH_FRONT_ING))
    {
        Shooting_Instance.Calc.PushRod.State = PUSH_BACK_ENTER;

        Shooting_Instance.Calc.PushRod.T_Angle = PUSHROD_POSTION_FRONT_DEG;
    }
    else if((Shooting_Instance.CMD.Shooting.Fire == OFF) && (Shooting_Instance.Calc.PushRod.State = PUSH_BACK_ING))
    {
        Shooting_Instance.Calc.PushRod.State = PUSH_FRONT_ENTER;

        Shooting_Instance.Calc.PushRod.T_Angle = PUSHROD_POSTION_BACK_DEG;
    }
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

static void Dial_Update_Target(void);
static void Dial_Load_StateMachine(void);

//** #################################################################################################### **//
//** ====================================== 对外若定义覆盖函数 =========================================== **//
//** #################################################################################################### **//

//初始化函数
void Shooting_Init(void)
{
    //初始模式STOP
    Shooting_Instance.CMD.ctrl = STOP_MODE;
    //初始化拨盘状态
    Shooting_Instance.Logic.LoadState = LOAD_STOP;

    PID_Init(&Dial_Motor_STOP,3.0f,0.0f,0.0f,-DJI_M3508_R,DJI_M3508_R,-5.0f, 5.0f);

	//拨盘PID
	PID_Init(&Dial_In,1.1f,0.0f,0.0f,-DJI_M3508_R,DJI_M3508_R,-10.0f, 10.0f);
	PID_Init(&Dial_Ex,50.0f,0.003f,50.0f,-10000,10000,-1000.0f, 1000.0f);

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
    Dial_Load_StateMachine();
    Dial_Update_Target();
}

//计算控制量
void Shooting_CtrlCalc(void)
{
    if(Shooting_Instance.Logic.LoadState == LOAD_ING)
    {
        Shooting_Instance.Calc.Dial.Ctrl_Vel = PID_Double_Calculate(&Dial_In,
                                                                    &Dial_Ex,
                                                                    Shooting_Instance.Calc.Dial.T_rpm,
                                                                    Shooting_Instance.DJI_Motordata.DIAL.current_ma,
                                                                    Shooting_Instance.DJI_Motordata.DIAL.speed_rpm,
                                                                    DIAL_PID_THRESHOLD);
    }
    else if((Shooting_Instance.Logic.LoadState == LOAD_STOP) && (Shooting_Instance.Logic.LoadState == LOAD_WAIT))
    {
        Shooting_Instance.Calc.Dial.Ctrl_Vel = 0;
    }

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

//更新拨盘目标量
static void Dial_Update_Target(void)
{
    if(Shooting_Instance.Logic.LoadState == LOAD_ING)
    {
        Shooting_Instance.Calc.Dial.T_Speed = DIAL_MAX_SPEED_M_S;
        Shooting_Instance.Calc.Dial.T_rpm = (int16_t)calc_motor_rpm_from_speed(Shooting_Instance.Calc.Dial.T_Speed,(DIAL_RADIUS_MM / 1000),DIAL_RATIO);
    }
    else if((Shooting_Instance.Logic.LoadState == LOAD_STOP) || (Shooting_Instance.Logic.LoadState == LOAD_WAIT))
    {
        Shooting_Instance.Calc.Dial.T_rpm = 0;
    }
}

//检测拨盘状态并设置标志
static void Dial_Load_StateMachine(void)
{
    // 获取当前状态
    LOAD_State_e currState = Shooting_Instance.Logic.LoadState;
    // 获取当前状态
    static uint32_t LoadOK_tick = 0;

    switch(currState)
    {
        // ====================== 停止状态 ======================
        case LOAD_STOP:
            // 触发条件：装弹命令有效
            if(Shooting_Instance.CMD.Shooting.Load == ON)
            {
                // 进入装弹中
                Shooting_Instance.Logic.LoadState = LOAD_ING;
            }
            break;

        // ====================== 装弹中 ======================
        case LOAD_ING:
            // 条件1：拨盘转速极低
            // 条件2：电机电流 >=5000mA（堵转/到位）
            if((Shooting_Instance.DJI_Motordata.DIAL.speed_rpm <= 10) &&
               (Shooting_Instance.DJI_Motordata.DIAL.current_ma >= 5000))
            {
                // 装弹完成
                Shooting_Instance.Logic.LoadState = LOAD_OK;
                LoadOK_tick = HAL_GetTick();
            }
            // 中途命令取消 → 回到停止
            else if(Shooting_Instance.CMD.Shooting.Load == OFF)
            {
                Shooting_Instance.Logic.LoadState = LOAD_STOP;
            }
            break;

        // ====================== 装弹完成 ======================
        case LOAD_OK:
            //当装填完毕持续时间为500ms时等待
            if(HAL_GetTick() - LoadOK_tick >= 500)
            {
                Shooting_Instance.Logic.LoadState = LOAD_WAIT;
            }
            break;

        // ====================== 等待复位 ======================
        case LOAD_WAIT:
            // 命令取消 → 回到停止，完成一次完整流程
            if(Shooting_Instance.CMD.Shooting.Load == OFF)
            {
                Shooting_Instance.Logic.LoadState = LOAD_STOP;
            }
            break;

        // ====================== 异常处理 ======================
        default:
            Shooting_Instance.Logic.LoadState = LOAD_STOP;
            break;
    }
}

#endif

#endif
