#include "Chassis_Move.h"

//** 函数一级 **//
//* 函数二级 *//
/* 函数名称或作用 */

//** 外部变量 **//
extern RC_Ctl_t RC_Ctl;
extern motor_measure_t motor_chassis[8];

//** 内容变量 **//

//底盘四个电机的PID
PID_DoubleDef PID_ChMotor1;
PID_DoubleDef PID_ChMotor2;
PID_DoubleDef PID_ChMotor3;
PID_DoubleDef PID_ChMotor4;
//定义目标方向
Target_Direction_Vel GTDV;//云台目标方向
Target_Direction_Vel CTDV;//底盘目标方向
//定义底盘四个电机速度
Chassis_Motor_Vel CM_TV;//目标速度
Chassis_Motor_Vel CM_SV;//实际设置速度
//定义底盘变量
Chassis_Variable Ch_Varia;
//定义底盘跟随PID
PID_DoubleDef PID_CHFollowing;

//** 本地函数声明 **//

/* 获取目标速度 */
static void  Get_TargetDirectionVel(void);
/* 计算底盘四个轮子的最终速度值 */
static void Get_ChassisMotorVel(void);
/* 计算麦轮速度 */
static void WheatWheel_Calculation(Target_Direction_Vel* pTD_Vel ,Chassis_Motor_Vel* pCM_Vel);
/* PID计算 */
static void Chassis_PID_Caculate(Chassis_Motor_Vel* pCM_TVel,Chassis_Motor_Vel* pCM_SVel);
/* 滤波函数 */
static float Filter_F(float Input,float Min,float Max);
/* 获取底盘相对云台角度 */
static void Chassis_Get_RelativeAngle(void);
/* 通过目标方向计算底盘前进方向 */
static void Get_TargetChassisDirectionVel(void);
/* 底盘跟随函数 */
static float Chassis_Following(void);

// ** 核心函数 ** //

//* 对外函数 *//
void Chassis_Init(void)
{
	PID_Init(&PID_ChMotor1.In_PID,1,0,0,-16384,16384,-10,10);		//内环电流（输出换）
	PID_Init(&PID_ChMotor1.Ex_PID,10,0,0,-5000,5000,-10,10);		//外环速度（目标换）
	
	//PID_Init(&PID_CHFollowing.In_PID,20,0,0,-16384,16384,-10,10);    	//内环速度
	PID_Init(&PID_CHFollowing.In_PID,50,0,0,-16384,16384,-10,10); 	//内环速度
	PID_Init(&PID_CHFollowing.Ex_PID,50,0,0,-5000,5000,-10,10); 	//外环角度
}
/* 设置底盘每个电机的速度 */
void Chassis_Motor_Set(void)
{
	Get_TargetChassisDirectionVel();//获取目标方向
	WheatWheel_Calculation(&CTDV,&CM_TV);//计算目标方向下麦轮速度
	Chassis_PID_Caculate(&CM_TV,&CM_SV);//计算PID
	CAN_cmd_chassis(CM_SV.F_R_Vel,CM_SV.F_L_Vel,CM_SV.B_L_Vel,CM_SV.B_R_Vel);//设置每个电机的速度
    //         1              2         3         4
    //       F_R            F_L       B_L		  B_R
}

//* 本地函数 *//

/* 获取目标速度 */
static void  Get_TargetDirectionVel(void)
{
	GTDV.FB_Vel = ((RC_Ctl.rc.ch3- 1024)*SPEED_RATIO);
	GTDV.RL_Vel = ((RC_Ctl.rc.ch2 - 1024)*SPEED_RATIO);
}	
/* 通过目标方向计算底盘前进方向 */
static void Get_TargetChassisDirectionVel(void)
{
	Get_TargetDirectionVel();
	Chassis_Get_RelativeAngle();
	Ch_Varia.Theta_Radian = Ch_Varia.RelativeAngle_Radian - Degrees_To_Radians(ENCODER_ZERO_OFFSET);
	Ch_Varia.Theta_Degree = Ch_Varia.RelativeAngle_Degree - ENCODER_ZERO_OFFSET;
	if(RC_Ctl.rc.s1 == RC_LEVER_MID)
	{
		CTDV.FB_Vel = GTDV.FB_Vel;
		CTDV.RL_Vel = GTDV.RL_Vel;
		CTDV.Rotation_Vel = -1.0 * (Chassis_Following());
	}
	if(RC_Ctl.rc.s1 == RC_LEVER_DOWN)
	{
		CTDV.RL_Vel = ((GTDV.RL_Vel * cos(Ch_Varia.Theta_Radian)) - (GTDV.FB_Vel * sin(Ch_Varia.Theta_Radian)));
		CTDV.FB_Vel = (GTDV.RL_Vel * sin(Ch_Varia.Theta_Radian)) + (GTDV.FB_Vel * cos(Ch_Varia.Theta_Radian));
		CTDV.Rotation_Vel = ROTATION_VALUE;
	}
}

/* 计算底盘四个轮子的最终速度值 */
static void Get_ChassisMotorVel(void)
{
	WheatWheel_Calculation(&CTDV,&CM_TV);
}
/* 计算麦轮速度 */
static void WheatWheel_Calculation(Target_Direction_Vel* pTD_Vel ,Chassis_Motor_Vel* pCM_Vel)
{
	pCM_Vel->F_L_Vel =  -((pTD_Vel->FB_Vel) - (pTD_Vel->RL_Vel) + (pTD_Vel->Rotation_Vel));  // 左前轮
	pCM_Vel->F_R_Vel =  (pTD_Vel->FB_Vel) + (pTD_Vel->RL_Vel) - (pTD_Vel->Rotation_Vel);  // 右前轮
	pCM_Vel->B_L_Vel =  (pTD_Vel->FB_Vel) - (pTD_Vel->RL_Vel) - (pTD_Vel->Rotation_Vel);  // 左后轮
	pCM_Vel->B_R_Vel =  -((pTD_Vel->FB_Vel) + (pTD_Vel->RL_Vel) + (pTD_Vel->Rotation_Vel));  // 右后轮
}
/* PID计算函数 */
static void Chassis_PID_Caculate(Chassis_Motor_Vel* pCM_TVel,Chassis_Motor_Vel* pCM_SVel)
{
	pCM_SVel->F_L_Vel = PID_Double_Caculate(&PID_ChMotor1.In_PID,&PID_ChMotor1.Ex_PID,pCM_TVel->F_L_Vel,motor_chassis[MOTOR_F_L].given_current,motor_chassis[MOTOR_F_L].speed_rpm,ERROR_MOTOR_CH);
	pCM_SVel->F_R_Vel = PID_Double_Caculate(&PID_ChMotor1.In_PID,&PID_ChMotor1.Ex_PID,pCM_TVel->F_R_Vel,motor_chassis[MOTOR_F_R].given_current,motor_chassis[MOTOR_F_R].speed_rpm,ERROR_MOTOR_CH);
	pCM_SVel->B_L_Vel = PID_Double_Caculate(&PID_ChMotor1.In_PID,&PID_ChMotor1.Ex_PID,pCM_TVel->B_L_Vel,motor_chassis[MOTOR_B_L].given_current,motor_chassis[MOTOR_B_L].speed_rpm,ERROR_MOTOR_CH);
	pCM_SVel->B_R_Vel = PID_Double_Caculate(&PID_ChMotor1.In_PID,&PID_ChMotor1.Ex_PID,pCM_TVel->B_R_Vel,motor_chassis[MOTOR_B_R].given_current,motor_chassis[MOTOR_B_R].speed_rpm,ERROR_MOTOR_CH);
}

/* 获取底盘和云台的相对角度（弧度制） */
static void Chassis_Get_RelativeAngle(void)
{
	Ch_Varia.RelativeAngle_Radian = motor_chassis[4].Radian_Angle;
	Ch_Varia.RelativeAngle_Degree = motor_chassis[4].Degree_Angle;
}
/* 底盘跟随函数 */
static float Chassis_Following(void)
{
	
//	float CAngle = Ch_Varia.Theta_Degree;
//	float CSpeed = motor_chassis[4].speed_rpm;
//	static float Target = 0.00f;
//	
//	return PID_Double_CycleAngle(&PID_CHFollowing.In_PID,&PID_CHFollowing.Ex_PID,Target,CSpeed,CAngle,ERROR_FOLLOWING);
	return PID_Calculate_CycleAngle(&PID_CHFollowing.In_PID,Ch_Varia.Theta_Degree,0.00f);
//	float CurrentDegree = MyMath_Limit_Float(Ch_Varia.Theta_Degree,-180.0,180.0,1);
//	return PID_Double_Caculate(&PID_CHFollowing.In_PID,&PID_CHFollowing.Ex_PID,0.0f,motor_chassis[4].speed_rpm,CurrentDegree,ERROR_FOLLOWING);

}


/* 滤波函数 */
static float Filter_F(float Input,float Min,float Max)
{
	if((Input < Max) && (Input > Min))
	{
		return 0;
	}
	else
	{
		return Input;
	}
}
///*motor data:  
//0:chassis motor1 3508;
//1:chassis motor3 3508;
//2:chassis motor3 3508;
//3:chassis motor4 3508;
//4:yaw gimbal motor 6020;
//5:pitch gimbal motor 6020;
//6:trigger motor 2006;  

//3508电机控制电流, 范围 [-16384,16384]
//6020电机控制电流, 范围 [-30000,30000]
//2006电机控制电流, 范围 [-10000,10000]
//820R,3510电机电调电流，范围[-32768~32767]
//*/

//#include "Chassis_Move.h"
//#include "Remote_Control.h"
//#include "Motors.h"
//#include "PID.h"
//#include "math.h"

//#define MY_PI 3.14159265358979323846

//extern RC_Ctl_t RC_Ctl;
//extern motor_measure_t motor_chassis[7];

//PID_HandleTypeDef ChassisPID_Motor1;
//PID_HandleTypeDef ChassisPID_Motor2;
//PID_HandleTypeDef ChassisPID_Motor3;
//PID_HandleTypeDef ChassisPID_Motor4;
//PID_HandleTypeDef CPID_FGBalAngle;
//PID_HandleTypeDef CPID_FGBalSpeed;

//Chassis_Cmd_Direction_Vel CCD_Vel;
//Chassis_Cmd_Direction_Vel GimbalCD_Vel;
//Chassis_Motor_Vel CM_Vel;

//float ChassisGimbal_RelativeAngle;
//float Theta;
//float RotationVel_Value = 500.0f;
//uint8_t SpeeRatio = 2;



//int16_t F_L_MVel = 0;//F_L 0x201
//int16_t F_R_MVel = 0;//F_R 0x202
//int16_t B_L_MVel = 0;//B_L 0x203
//int16_t B_R_MVel = 0;//B_R 0x204

//// 定义编码器零位偏移值
//#define ENCODER_ZERO_OFFSET 283.0f

//// 将编码器原始角度转换为以315度为零点的Theta角度
//float convertEncoderAngle(float encoder_angle) {
//    // 计算偏移后的角度
//    float theta = fmod(encoder_angle - ENCODER_ZERO_OFFSET, 360.0f);
//    
//    // 确保角度在[0, 360)范围内
//    if (theta < 0) {
//        theta += 360.0f;
//    }
//    
//    return theta;
//}

//float Degrees_To_Radians(float degrees) {
//    return degrees * MY_PI / 180.0;
//}

//void Get_ChassisGimbal_RelativeAngle()
//{
//	ChassisGimbal_RelativeAngle = motor_chassis[4].Radian_Angle;
//}

//void  Get_GimBalDirectionVel(void)
//{
//	GimbalCD_Vel.FB_Vel = ((RC_Ctl.rc.ch3- 1024)*SpeeRatio);
//	GimbalCD_Vel.RL_Vel = ((RC_Ctl.rc.ch2 - 1024)*SpeeRatio);
//	GimbalCD_Vel.Rotation_Vel = (RC_Ctl.rc.ch0 - 1024)*SpeeRatio;
//}	

//float Chassis_YawPIDCalculation(float Target_Angle,PID_HandleTypeDef* pPID_Angle,PID_HandleTypeDef* pPID_Speed){
//	float PID_ExOutPut;
//	float PID_InOutPut;
//	
//	pPID_Angle->Target = Target_Angle;
//	pPID_Angle->Current = motor_chassis[4].Degree_Angle - ENCODER_ZERO_OFFSET;
//	PID_ExOutPut = PID_Angle_Calculate(pPID_Angle,pPID_Angle->Current,1);
//	
//	pPID_Speed->Target = PID_ExOutPut;
//	pPID_Speed->Current = motor_chassis[4].speed_rpm;
//	
//	if(fabs((pPID_Speed->Target)- (pPID_Speed->Current))< 1.0f)
//	{
//		PID_InOutPut = 0;
//	}
//	else
//	{
//		PID_InOutPut = PID_Speed_Calculate(pPID_Speed,pPID_Speed->Current,1);
//	}
//	
//	return PID_InOutPut;
//}

//void  Get_ChassisDirectionVel(Chassis_Cmd_Direction_Vel* pCCD_Vel)
//{
//	Get_GimBalDirectionVel();
//	Get_ChassisGimbal_RelativeAngle();
//	Theta = ChassisGimbal_RelativeAngle - Degrees_To_Radians(ENCODER_ZERO_OFFSET);
//	if(RC_Ctl.rc.s1 == 2)
//	{
//		pCCD_Vel->RL_Vel = ((GimbalCD_Vel.RL_Vel * cos(Theta)) - (GimbalCD_Vel.FB_Vel * sin(Theta)));
//		pCCD_Vel->FB_Vel = (GimbalCD_Vel.RL_Vel * sin(Theta)) + (GimbalCD_Vel.FB_Vel * cos(Theta));
//		pCCD_Vel->Rotation_Vel = RotationVel_Value;
//	}
//	else if(RC_Ctl.rc.s1 == 3)
//	{
//		pCCD_Vel->FB_Vel = GimbalCD_Vel.FB_Vel;
//		pCCD_Vel->RL_Vel = GimbalCD_Vel.RL_Vel;
//		pCCD_Vel->Rotation_Vel =(Chassis_YawPIDCalculation(0,&CPID_FGBalAngle,&CPID_FGBalSpeed));//Chassis Following GimBal
//	}
//}

//void WheatWheel_Calculation(Chassis_Cmd_Direction_Vel* pCCD_Vel ,Chassis_Motor_Vel* pCM_Vel)
//{
//	pCM_Vel->F_L_Vel = ((pCCD_Vel->FB_Vel) + (pCCD_Vel->RL_Vel) + (pCCD_Vel->Rotation_Vel));
//	pCM_Vel->F_R_Vel = -((pCCD_Vel->FB_Vel) - (pCCD_Vel->RL_Vel) - (pCCD_Vel->Rotation_Vel));
//	pCM_Vel->B_L_Vel = ((pCCD_Vel->FB_Vel) - (pCCD_Vel->RL_Vel) + (pCCD_Vel->Rotation_Vel));
//	pCM_Vel->B_R_Vel = -((pCCD_Vel->FB_Vel) + (pCCD_Vel->RL_Vel) - (pCCD_Vel->Rotation_Vel));
//}

//float WheatWheel_PIDCalculation(PID_HandleTypeDef* pChassisPID,int16_t Target_Speed,int16_t Curent_Speed)
//{
//	pChassisPID->Target = Target_Speed;
//	PID_Speed_Calculate(pChassisPID,Curent_Speed,1);
//	return (pChassisPID->Output);
//}

//void Chassis_Move_MotorSet(void)
//{
//	Get_ChassisDirectionVel(&CCD_Vel);
//	WheatWheel_Calculation(&CCD_Vel,&CM_Vel);
//	
//	F_L_MVel = WheatWheel_PIDCalculation(&ChassisPID_Motor1,CM_Vel.F_L_Vel,motor_chassis[0].speed_rpm);
//	F_R_MVel = WheatWheel_PIDCalculation(&ChassisPID_Motor2,CM_Vel.F_R_Vel,motor_chassis[1].speed_rpm);	
//	B_L_MVel = WheatWheel_PIDCalculation(&ChassisPID_Motor3,CM_Vel.B_L_Vel,motor_chassis[2].speed_rpm);
//	B_R_MVel = WheatWheel_PIDCalculation(&ChassisPID_Motor4,CM_Vel.B_R_Vel,motor_chassis[3].speed_rpm);


//	CAN_cmd_chassis(F_L_MVel,F_R_MVel,B_L_MVel,B_R_MVel);
//	//              1              2         3         4
//	//             F_R            F_L       B_L		  B_R
//}

//void Chassis_Init(void)
//{
//	PID_Init(&ChassisPID_Motor1, 20, 0, 0, -2000, 2000, -10, 10);
//	PID_Init(&ChassisPID_Motor2, 20, 0, 0, -2000, 2000, -10, 10);
//	PID_Init(&ChassisPID_Motor3, 20, 0, 0, -2000, 2000, -10, 10);
//	PID_Init(&ChassisPID_Motor4, 20, 0, 0, -2000, 2000, -10, 10);
//	PID_Init(&CPID_FGBalAngle, 20, 0, 0, -50, 50, -10, 10);
//	PID_Init(&CPID_FGBalSpeed, 15, 0, 0, -3000, 3000, -10, 10);
//	CAN_cmd_chassis(0.0f,0.0f,0.0f,0.0f);
//}


