//#include "Friction_Wheel_Shooting.h"



//#define NEED_ACCUM_ANGEL 1500.0f //6900.98f

//extern RC_Ctl_t RC_Ctl;
//extern uint8_t BOARD_ID_Int;
//extern int16_t Dial_Vel;
//extern motor_measure_t motor_chassis[7];

//float FWS_OutPutL;
//float FWS_OutPutR;
//float FWS_OutPutD;

//PID_HandleTypeDef Friction_Speed_PID;

//PID_HandleTypeDef Dial_AnglePID;
//PID_HandleTypeDef Dial_SpeedPID;

//float Dial_InitAngle = 0.0f;
//float Dial_TargetAngle = 0.0f;
//float Dial_AddAngle = 60.0f;
//uint8_t Dial_Flag = 0;

//float Last_EncAngle;
//float Crent_EncAngle;

//float RelativeAngle = 0;

//extern float Convert_Angle(float degrees);

//float DialAngleMove(float Relative);
//float DialMoveFlag = 0;



//float ShootOption_SpeedPID(PID_HandleTypeDef* pChassisPID,int16_t Target_Speed,int16_t Curent_Speed){
//	pChassisPID->Target = Target_Speed;
//	PID_Speed_Calculate(pChassisPID,Curent_Speed,1);
//	return (pChassisPID->Output);
//}


////float Dial_AnglePID(PID_HandleTypeDef* pChassisPID,int16_t Target_Angle,int16_t Curent_Angle)
////{
////	pChassisPID->Target = Target_Angle;
////	PID_Speed_Calculate(pChassisPID,Curent_Angle,1);
////	return (pChassisPID->Output);
////}

////float DialAngle_PIDCalculation(float Target_Angle){
////	float PID_ExOutPut;
////	float PID_InOutPut;
////	
////	Dial_Angle_PID.Target = Convert_Angle(Target_Angle);
////	Dial_Angle_PID.Current = motor_chassis[6].Degree_Angle;
////	PID_ExOutPut = PID_Angle_Calculate(&Dial_Angle_PID,Dial_Angle_PID.Current,1);
////	
////	Dial_Speed_PID.Target = PID_ExOutPut;
////	Dial_Speed_PID.Current = motor_chassis[4].speed_rpm;
////	
////	if(fabs(Dial_Angle_PID.Target - Dial_Angle_PID.Current)< 0.1f)
////	{
////		PID_InOutPut = 0;
////	}
////	else
////	{
////		PID_InOutPut = PID_Speed_Calculate(&Dial_Speed_PID,Dial_Speed_PID.Current,1);
////	}
////	return PID_InOutPut;
////}


//void reset_accumulated_angle(void);

//void GimBal_ShootOption(void){
//	static uint16_t tempDelay;
//	
//	if(RC_Ctl.rc.s2 == 2)
//	{
//		
//		
//		Dial_Vel = (RC_Ctl.key.v - 1024) * 25;
//		
//		if(Dial_Vel >= 3000)
//		{
//			Dial_Vel = 3000;
//		}
//		else if(Dial_Vel <= 0)
//		{
//			Dial_Vel = 0;
//			Dial_Flag = 0;
//		}
//		if(BOARD_ID_Int == 1)
//		{
//			Dial_Vel = Dial_Vel * 10 ;
//			FWS_OutPutL = ShootOption_SpeedPID(&Friction_Speed_PID,Dial_Vel,motor_chassis[0].speed_rpm);
//			FWS_OutPutR = ShootOption_SpeedPID(&Friction_Speed_PID,-Dial_Vel,motor_chassis[1].speed_rpm);
//			CAN_cmd_chassis((FWS_OutPutL),(FWS_OutPutR),0,0);//
//		}
//		if(BOARD_ID_Int == 2)
//		{

//			if(Dial_Flag == 0 && Dial_Vel != 0 && DialMoveFlag == 0)
//			{
//				reset_accumulated_angle();
//				RelativeAngle = NEED_ACCUM_ANGEL;
//				Dial_Flag = 1;
//			}
//			else if(Dial_Flag == 1 && Dial_Vel == 0)
//			{
//				Dial_Flag = 0;
//				
//			}
//			FWS_OutPutD = DialAngleMove(RelativeAngle);
//			
//		}
//	}
//	else if(RC_Ctl.rc.s2 == 3)
//	{
//		Dial_Vel = 0;
//	}
//}








//float PID_F_Calculate(PID_HandleTypeDef *pid, float Current_Value, float dt){
//	
//    if(pid == NULL || dt <= 0) return 0.0f;
//    
//    // 保存当前值
//    pid->Current = Current_Value;
//    
//    // 计算当前误差
//    float error = pid->Target - Current_Value;
//    
//    // 计算积分项并限幅
//    pid->Integral += error * dt;
//    if(pid->Integral > pid->Integral_Max) pid->Integral = pid->Integral_Max;
//    if(pid->Integral < pid->Integral_Min) pid->Integral = pid->Integral_Min;
//    
//    // 计算微分项(当前误差与上次误差的差值)
//    float derivative = (error - pid->Prev_Error) / dt;
//    
//    // 计算PID输出
//    pid->Output = pid->Kp * error + pid->Ki * pid->Integral + pid->Kd * derivative;
//	
//    // 输出限幅
//    if(pid->Output > pid->Output_Max) pid->Output = pid->Output_Max;
//    if(pid->Output < pid->Output_Min) pid->Output = pid->Output_Min;
//    
//    // 保存本次误差用于下次计算
//    pid->Prev_Error = error;
//    
//    return (float)(pid->Output);
//}


//float Dial_PIDCalculate(float Target,float Curent,float Curent_Speed)
//{
//	float PID_ExOutput;
//	float PID_InOutput;
//		
//	Dial_AnglePID.Target = Target;
//	Dial_AnglePID.Current = Curent;
//	PID_ExOutput = PID_F_Calculate(&Dial_AnglePID,Dial_AnglePID.Current,1);
//	
//	Dial_SpeedPID.Target = PID_ExOutput;
//	Dial_SpeedPID.Current = Curent_Speed;
//	
//	if(fabs(Dial_AnglePID.Prev_Error) < 1.0f)
//	{
//		PID_InOutput = 0;
//	}
//	else 
//	{
//		PID_InOutput = PID_F_Calculate(&Dial_SpeedPID,Dial_SpeedPID.Current,1);
//		DialMoveFlag = 1;
//	}
//	if(fabs(Dial_AnglePID.Prev_Error) < 10.0f)
//	{
//		DialMoveFlag = 0;
//	}
//	
//	
//	return PID_InOutput;
//}

//#include <stdio.h>
//#include <math.h>

//// 静态变量用于保存历史状态
//static double s_last_angle = -1.0;       // 上一次角度，-1表示未初始化
//static double s_accumulated_angle = 0.0; // 累加角度值
//static const double THRESHOLD = 270.0;   // 跳变检测阈值

///**
// * @brief 重置累加角度为0
// */
//void reset_accumulated_angle(void) {
//    s_accumulated_angle = 0.0;
//    // 保持当前角度作为新的基准点
//    // 如果已经初始化过
//    if (s_last_angle >= 0.0) {
//        // 假设当前角度就是上次记录的角度
//        // 实际应用中可以考虑传入当前角度作为新基准
//    }
//}

///**
// * @brief 计算累加角度
// * @param current_angle 当前角度(0-360度)
// * @return 累加角度值
// */
//double get_accumulated_angle(double current_angle) {
//    // 确保角度在0-360度范围内
//    current_angle = fmod(fmod(current_angle, 360.0) + 360.0, 360.0);
//    
//    // 首次初始化
//    if (s_last_angle < 0.0) {
//        s_last_angle = current_angle;
//        return s_accumulated_angle; // 初始累加值为0
//    }
//    
//    // 计算角度差
//    double angle_diff = current_angle - s_last_angle;
//    
//    // 处理正转整圈情况（如350° -> 10°，实际增加20°）
//    if (angle_diff < -THRESHOLD) {
//        s_accumulated_angle += (angle_diff + 360.0);
//    }
//    // 处理反转整圈情况（如10° -> 350°，实际减少20°）
//    else if (angle_diff > THRESHOLD) {
//        s_accumulated_angle += (angle_diff - 360.0);
//    }
//    // 正常角度变化
//    else {
//        s_accumulated_angle += angle_diff;
//    }
//    
//    // 更新上一次角度
//    s_last_angle = current_angle;
//    
//    return s_accumulated_angle;
//}


//float DialAngleMove(float Relative)
//{
//		Crent_EncAngle = (double)motor_chassis[6].Degree_Angle;

//		float Temp = get_accumulated_angle(Crent_EncAngle);
//		float Output = Dial_PIDCalculate(Relative,Temp,motor_chassis[6].speed_rpm);
//		
//		return Output;
//}

//void Friction_Shooting_Init(void)
//{
//	PID_Init(&Friction_Speed_PID, 50, 0, 0, -20000, 20000, -10, 10);
//	
//	PID_Init(&Dial_AnglePID, 30, 0, 0, -2000, 2000, -10, 10);
//	PID_Init(&Dial_SpeedPID, 5, 0, 0, -5000, 5000, -10, 10);
//}
