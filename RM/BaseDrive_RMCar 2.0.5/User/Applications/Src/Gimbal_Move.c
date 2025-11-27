#include "Gimbal_Move.h"

PID_DoubleDef PID_GimBal_Yaw;
PID_DoubleDef PID_GimBal_Pitch;

//** 外部变量 **//
extern RC_Ctl_t RC_Ctl;
extern motor_measure_t motor_chassis[8];
extern float Board1_IMUDatas[3];
extern float IMU_DegAngle[3];
//** 内部变量 **//

Gimbal_Target_Direction GD_TV;//目标Yaw 和 Pitch的值 
Gimbal_Target_Direction GD_SV;//设置Yaw 和 Pitch的值
GimBal_Variable_t GB_Varia;
//** 本地函数声明 **//

/* 获取目标方向 */
static void Gimbal_Get_Direction(void);
/* 获取Yaw和Pitch控制角度 */
static void Gimbal_Yaw_Contral_Angle(void);
/* Yaw轴自稳 */
static float GimBal_YawStable(void);
/* Pitch轴自稳 */
static float GmBal_PitchStable(void);
/* 初始化Yaw归零 */
static void GimBal_YawInit(float* PID_Out);
//** 核心函数 **//

//* 对外函数 *//
/* 云台初始化 */
void Gimbal_Init(void)
{
	PID_Init(&PID_GimBal_Yaw.In_PID,250,0,0,-16384,16384,-10,10);//内环速度环，输出环
	PID_Init(&PID_GimBal_Yaw.Ex_PID,5,0,0,-2000,2000,-10,10);//外环角度环，目标环
	
	PID_Init(&PID_GimBal_Pitch.In_PID,10,0,0,-16384,16384,-10,10);//内环速度环，输出环
	PID_Init(&PID_GimBal_Pitch.Ex_PID,20,0,0,-2000,2000,-10,10);//外环角度环，目标环
	GB_Varia.Yaw_InitFlag = 0;
	GB_Varia.YawTheta = 0;
}

/* 云台电机设置 */
void GimBal_Yaw_Motor_Set(float Dial,float Salute)
{
	Gimbal_Get_Direction();
	
	if(GB_Varia.Yaw_InitFlag)
	{
		GD_SV.Yaw = GimBal_YawStable();
	}
	else if(GB_Varia.Yaw_InitFlag == 0)
	{
		
		GimBal_YawInit(&GD_SV.Yaw);
	}
	CAN_cmd_gimbal(GD_SV.Yaw,GD_SV.Yaw,(int16_t)Dial,(int16_t)Salute);
}

float GimBal_Pitch_Motor_Set(void)
{
	Gimbal_Get_Direction();
	
	GD_SV.Pitch = -(GmBal_PitchStable());
	
	CAN_cmd_gimbal(0,0,GD_SV.Pitch,GD_SV.Pitch);
}

//* 本地函数 *//

/* 获取目标方向 */
static void Gimbal_Get_Direction(void)
{
	GD_TV.Yaw = -(RC_Ctl.rc.ch0 - 1024);
	GD_TV.Pitch = -(RC_Ctl.rc.ch1 - 1024);
}

/* 云台Yaw自稳 */
static float GimBal_YawStable(void){

	static float TYaw = 0.0f;
	static float CYaw = 0.0f;
	static float CSpeed = 0.0f;
	
	TYaw = TYaw + MyMath_Map_Range(GD_TV.Yaw,-RC_CH_RANGE,RC_CH_RANGE,-YAW_ADDVALUE,YAW_ADDVALUE);
	TYaw = MyMath_Limit_Float(TYaw,-180.00f,180.00f,1);
	
	CYaw = MyMath_Limit_Float(Board1_IMUDatas[0],-180.00f,180.00f,1);
	CSpeed = motor_chassis[4].speed_rpm;
	
	return PID_Calculate_CycleAngle(&PID_GimBal_Yaw.In_PID,CYaw,TYaw);
}

/* 云台Yaw轴初始化归零，到达 */
static void GimBal_YawInit(float* PID_Out)
{
	static uint8_t add;
	GB_Varia.YawTheta = motor_chassis[4].Degree_Angle - ENCODER_ZERO_OFFSET;
		
	if(fabs(GB_Varia.YawTheta) < 5.0f)
	{
		add++;
		if(add>100)
		{
			GB_Varia.Yaw_InitFlag = 1;
		}
	}
	else
	{
		*PID_Out = PID_Calculate_CycleAngle(&PID_GimBal_Yaw.In_PID,GB_Varia.YawTheta,0);
	}	
}

float test;
/* 云台Pitch自稳 */
static float GmBal_PitchStable(void)
{
	static float TPitch = 180.0f;
	static float CPitch = 0.0f;
	static float CSpeed = 0.0f;
	
	TPitch = TPitch + MyMath_Map_Range(GD_TV.Pitch,-RC_CH_RANGE,RC_CH_RANGE,-PITCH_ADDVALUE,PITCH_ADDVALUE);
	TPitch = MyMath_Limit_Float(TPitch,PITCH_RANGE_MIN,PITCH_RANGE_MAX,0);
	
	CPitch = convert_angle(IMU_DegAngle[2]);
	CSpeed = motor_chassis[7].speed_rpm;
	
	//return PID_Calculate(&PID_GimBal_Pitch.In_PID,CPitch,TPitch);
	return PID_Double_Caculate(&PID_GimBal_Pitch.In_PID,&PID_GimBal_Pitch.Ex_PID,TPitch,CSpeed,CPitch,0.1f);
}

/* 计算PID */

///*motor data:  
//0:chassis motor1 3508;
//1:chassis motor3 3508;
//2:chassis motor3 3508;
//3:chassis motor4 3508;
//4:yaw gimbal motor 6020;
//5:pitch gimbal motor 6020;
//6:Dial motor 3508拨盘; 
//7:Shoot Motor 2006;

//3508电机控制电流, 范围 [-16384,16384]
//6020电机控制电流, 范围 [-16384,16384]
//2006电机控制电流, 范围 [-10000,10000]
//820R,3510电机电调电流，范围[-32768~32767]
//*/

//// 定义编码器零位偏移值
//#define ENCODER_ZERO_OFFSET 283.0f

//#define INITPITCHANGLE 0

//extern RC_Ctl_t RC_Ctl;
//extern motor_measure_t motor_chassis[7];
//extern float RotationVel_Value;
//extern float IMU_Yaw;								//偏航角
//extern float IMU_Pitch;								//俯仰角
//extern float Board1_IMUDatas[3];
//extern float IMU_DegAngle[3];
//extern uint8_t BOARD_ID_Int;


//extern float FWS_OutPutR;
//extern float FWS_OutPutL;
//extern float FWS_OutPutD;



//Gimbal_Cmd_Direction_Vel GCD_Vel;

//PID_HandleTypeDef GimbalPID_Yaw_Speed;
//PID_HandleTypeDef GimbalPID_Yaw_Angle;
//PID_HandleTypeDef GimbalPID_Pitch_Speed;
//PID_HandleTypeDef GimbalPID_Pitch_Angle;
//PID_HandleTypeDef GimbalPID_Shoot_Motor;



//int16_t Yaw_Vel = 0;
//int16_t Pitch_Vel = 0;
//int16_t Dial_Vel = 0;
//float YawCurentAngel = 0;
//float PitchCurentAngel = 0;


//float LimtPitchAngle(float i)
//{	
//	if(i > 30){i = 30.0f;}
//	else if(i < -30){i = -30.0f;}
//	return i;
//}

//float Angle_Mapping(float degrees)
//{
//	float TempAngle;
//	if(degrees >= 0)
//	{
//		TempAngle = 180 - degrees;
//	}
//	else
//	{
//		TempAngle = fabs(degrees) - 180;
//	}
//	
//	return TempAngle;
//}

////float ReEncLimitTo_180Deg(float i) {
////    // 输入范围：[-660, 660] → 输出范围：[-180, 180]
////    return (i + 660.0f) * (3.0f/11.0f) - 180.0f;
////}

//float Convert_Angle(float degrees) {
//    // 处理0-360度到-180到180度的转换
//    if (degrees > 180.0f) {
//        return degrees - 360.0f;
//    } else {
//        return degrees;
//    }
//}

//double Normalize_Angle(double angle) {
//    // 加上足够大的360倍数确保结果为正
//    double positive_angle = angle + 360.0 * 1000.0;
//    // 对360取模，将角度限制在0-360度
//    double mod_angle = fmod(positive_angle, 360.0);
//    // 转换到-180到180度范围
//    if (mod_angle > 180.0) {
//        return mod_angle - 360.0;
//    } else {
//        return mod_angle;
//    }
//}

//void Get_GimbalDirectionVel(void){
//	GCD_Vel.Yaw = (RC_Ctl.rc.ch0 - 1024);
//	GCD_Vel.Pitch =(RC_Ctl.rc.ch1 - 1024);
//}

//float GimBal_YawSpeedPIDCalculation(PID_HandleTypeDef* pChassisPID,int16_t Target_Speed,int16_t Curent_Speed){
//	pChassisPID->Target = Target_Speed;
//	PID_Speed_Calculate(pChassisPID,Curent_Speed,1);
//	return (pChassisPID->Output);
//}

//float GimBal_YawPIDCalculation(float Target_Angle){
//	float PID_ExOutPut;
//	float PID_InOutPut;
//	
//	GimbalPID_Yaw_Angle.Target = Target_Angle;
//	GimbalPID_Yaw_Angle.Current = Convert_Angle(Board1_IMUDatas[0]);
//	PID_ExOutPut = PID_Angle_Calculate(&GimbalPID_Yaw_Angle,GimbalPID_Yaw_Angle.Current,1);
//	
//	GimbalPID_Yaw_Speed.Target = PID_ExOutPut;
//	GimbalPID_Yaw_Speed.Current = motor_chassis[4].speed_rpm;
//	
//	if(fabs(GimbalPID_Yaw_Angle.Target - GimbalPID_Yaw_Angle.Current)< 0.1f)
//	{
//		PID_InOutPut = 0;
//	}
//	else
//	{
//		PID_InOutPut = PID_Speed_Calculate(&GimbalPID_Yaw_Speed,GimbalPID_Yaw_Speed.Current,1);
//	}
//	return PID_InOutPut;
//}

//float GimBal_PitchPIDCalculation(float Target_Angle){
//	float PID_ExOutPut;
//	float PID_InOutPut;
//	
//	GimbalPID_Pitch_Angle.Target = Target_Angle;
//	GimbalPID_Pitch_Angle.Current = Angle_Mapping(Convert_Angle(IMU_DegAngle[2]));
//	PID_ExOutPut = PID_Angle_Calculate(&GimbalPID_Pitch_Angle,GimbalPID_Pitch_Angle.Current,1);
//	
//	GimbalPID_Pitch_Speed.Target = PID_ExOutPut;
//	GimbalPID_Pitch_Speed.Current = motor_chassis[6].speed_rpm;
//	
//	if(fabs(GimbalPID_Pitch_Angle.Target - GimbalPID_Pitch_Angle.Current)< 0.1f)
//	{
//		PID_InOutPut = 0;
//	}
//	else
//	{
//		PID_InOutPut = PID_Speed_Calculate(&GimbalPID_Pitch_Speed,GimbalPID_Pitch_Speed.Current,1);
//	}
//	return PID_InOutPut;
//}

//float GimBal_ShootSpeedPIDCalculation(PID_HandleTypeDef* pChassisPID,int16_t Target_Speed,int16_t Curent_Speed){
//	pChassisPID->Target = Target_Speed;
//	PID_Speed_Calculate(pChassisPID,Curent_Speed,1);
//	return (pChassisPID->Output);
//}

//void GimBal_YawStable(void){
//	
//	static float temp = 0;
//	if(GCD_Vel.Yaw != 1024)
//	{
//		temp++;
//	}
//	if(temp > 5)
//	{
//		YawCurentAngel = YawCurentAngel - (GCD_Vel.Yaw)/(1000);
//		YawCurentAngel = Normalize_Angle(YawCurentAngel);
//		Yaw_Vel = GimBal_YawPIDCalculation(YawCurentAngel);
//		temp = 0;
//	}
//}

//void GimBal_PitchStable(void){
//	PitchCurentAngel = PitchCurentAngel + (GCD_Vel.Pitch)/(3000);
//	PitchCurentAngel = LimtPitchAngle(PitchCurentAngel);
//	Pitch_Vel = GimBal_PitchPIDCalculation(PitchCurentAngel);
//	Pitch_Vel = Pitch_Vel;
//}


//float InitYawFlag = 0;
//float RxInitYawFlag = 0;
//extern uint8_t BIM088_ReSetFlag;

//uint16_t TimeDelay = 0;

//float GimBalYaw_PIDInit(float Target_Angle){
//	float PID_ExOutPut;
//	float PID_InOutPut;
//	
//	GimbalPID_Yaw_Angle.Target = Target_Angle;
//	GimbalPID_Yaw_Angle.Current = motor_chassis[4].Degree_Angle;
//	PID_ExOutPut = PID_Angle_Calculate(&GimbalPID_Yaw_Angle,GimbalPID_Yaw_Angle.Current,1);
//	
//	GimbalPID_Yaw_Speed.Target = PID_ExOutPut;
//	GimbalPID_Yaw_Speed.Current = motor_chassis[4].speed_rpm;
//		
//	if(fabs(GimbalPID_Yaw_Angle.Prev_Error) < 1.0f)
//	{
//		PID_InOutPut = 0;
//		TimeDelay++;
//		if(TimeDelay > 50)
//		{
//			InitYawFlag = 1;
//			TimeDelay = 0;
//		}
//	}
//	else
//	{
//		PID_InOutPut = PID_Speed_Calculate(&GimbalPID_Yaw_Speed,GimbalPID_Yaw_Speed.Current,1);
//	}
//	CAN_cmd_gimbal(PID_InOutPut,PID_InOutPut,0,0);

//}

//void GimBal_Move_MotorSet(void){
//	
//	if(RxInitYawFlag == 0 && BOARD_ID_Int == 1)
//	{
//		GimBalYaw_PIDInit(ENCODER_ZERO_OFFSET);
//	}
//	else if(RxInitYawFlag == 1 && BOARD_ID_Int == 1)
//	{
//		Get_GimbalDirectionVel();
//		
//		if(BIM088_ReSetFlag == 1)
//		{
//			GimBal_PitchStable();
//		}
//		
//		GimBal_ShootOption();
//		CAN_cmd_gimbal(0,0,Pitch_Vel,Pitch_Vel);
//	}
//	if(InitYawFlag == 0 && BOARD_ID_Int == 2)
//	{
//		GimBalYaw_PIDInit(ENCODER_ZERO_OFFSET);
//	}
//	else if(InitYawFlag == 1 && BOARD_ID_Int == 2)
//	{
//		Get_GimbalDirectionVel();
//		
//		if(BIM088_ReSetFlag == 1)
//		{
//			GimBal_YawStable();
//		}
//		
//		GimBal_ShootOption();
//		CAN_cmd_gimbal((Yaw_Vel),(Yaw_Vel),FWS_OutPutD,0);
//	}
//}


//	
//void Gimbal_Init(void)
//{
//	PID_Init(&GimbalPID_Yaw_Angle, 35, 0.01, 0.2, -400, 400, -5000, 5000);
//	PID_Init(&GimbalPID_Yaw_Speed, 20, 0, 0, -5000, 5000, -5000, 5000);
//	
//	PID_Init(&GimbalPID_Pitch_Angle, 30, 0, 0, -300, 300, -16384, 16384);
//	PID_Init(&GimbalPID_Pitch_Speed, 20, 0, 0, -16384, 16384, -16384, 16384);
//	
//	PID_Init(&GimbalPID_Shoot_Motor, 10, 1, 0, -16384, 16384, -16384, 16384);

//	Yaw_Vel = GimBal_YawPIDCalculation(YawCurentAngel);
//	CAN_cmd_gimbal(Yaw_Vel,Yaw_Vel,0,0);
//}


