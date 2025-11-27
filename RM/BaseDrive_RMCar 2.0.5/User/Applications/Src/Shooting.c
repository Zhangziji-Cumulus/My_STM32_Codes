#include "Shooting.h"

//** 外部变量 **//
extern RC_Ctl_t RC_Ctl;
extern motor_measure_t motor_chassis[8];

//** 本地变量 **//
PID_DoubleDef PID_SD;//拨盘
PID_Triple	PID_TSD;

PID_DoubleDef PID_SF_L;//摩擦轮
PID_DoubleDef PID_SF_R;//摩擦轮
PID_DoubleDef PID_SS;//礼炮机构
Shooting_Variable_t SH_Varia;//发射机构变量

//**声明本地函数**//

/* 更新拨盘相关变量 */
static float Update_Dial_Variable(void);
/* 拨盘PID计算 */
static float Dial_PID(float Add_Angle);
/* 更新摩擦轮数据 */
static float Update_Friction_Variable(void);
/* 更新彩弹发射 */
static float Update_Salute_Variable(void);

//** 对外函数 **//
void Shooting_Init(void)
{
	PID_Init(&PID_SD.In_PID,10,0,0,-16384,16384,-10,10);//内环速度，输出
	PID_Init(&PID_SD.Ex_PID,10,0,0,-2000,2000,-10,10);//外环角度，目标
	
	PID_Init(&PID_TSD.In_PID,1.5,0.01,0.05,-16384,16384,-10,10);
	PID_Init(&PID_TSD.Min_PID,20,0.01,0.1,-10000,10000,-10,10);
	PID_Init(&PID_TSD.Ex_PID,1.2,0.15,0.2,-500,500,-10,10);
	
	SH_Varia.Dial_AcumulateAngle = 0.0f;//初始化拨盘累加角度
	
	//SH_Varia.Salute_TAngle = 0;
	SH_Varia.Salute_AcumulateAngle = 0;
	
	
	PID_Init(&PID_SF_L.In_PID,1,0,0,-16384,16384,-10,10);//内环电流，输出
	PID_Init(&PID_SF_L.Ex_PID,50,0,0,-2000,2000,-10,10);//外环速度，目标
	
	PID_Init(&PID_SF_R.In_PID,1,0,0,-16384,16384,-10,10);//内环电流，输出
	PID_Init(&PID_SF_R.Ex_PID,50,0,0,-2000,2000,-10,10);//外环速度，目标
	
	PID_Init(&PID_SS.In_PID,2,0,0,-30000,30000,-10,10);//内环速度，输出
	PID_Init(&PID_SS.Ex_PID,500,0,0,-50000,50000,-10,10);//外环角度，目标
	
}

////* 发射结构拨盘部分，返回PID计算后的输出值 *//
//float Shooting_Dial(void)
//{
//	Update_Dial_Variable();//更新拨盘变量  
//	static uint8_t Dial_FlagState = 0;//发射机构状态
//	
//	float CSpeed = motor_chassis[6].speed_rpm;//更新电机当前速度
//	
//	if((fabs(RC_Ctl.key.v - 1024.0) > 1.0f) && (Dial_FlagState == 0))
//	{
//		SH_Varia.Dial_TAngle = SH_Varia.Dial_TAngle + (SH_Varia.Dial_IsF == 1 ? 1:-1 ) * (NEED_ACCUM_ANGEL);//SH_Varia.Dial_ADDAngle;//要移动的角度
//		Dial_FlagState = 1;
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
//	}
//	else if(Dial_FlagState == 1 && (fabs(RC_Ctl.key.v - 1024.0) < 1.0f))
//	{
//		Dial_FlagState = 0;
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
//	}
//		
//	return  PID_Double_Caculate(&PID_SD.In_PID,&PID_SD.Ex_PID,SH_Varia.Dial_TAngle,CSpeed,SH_Varia.Dial_AcumulateAngle,1.0f);
//}

float Shooting_Dial(void)
{
	Update_Dial_Variable();//更新拨盘变量  
	static uint8_t Dial_FlagState = 0;//发射机构状态
	
	float CSpeed = motor_chassis[6].speed_rpm;//更新电机当前速度
	float CCurrent = motor_chassis[6].given_current;
	
	if((fabs(RC_Ctl.key.v - 1024.0) > 1.0f) && (Dial_FlagState == 0))
	{
		SH_Varia.Dial_TAngle = SH_Varia.Dial_TAngle + (SH_Varia.Dial_IsF == 1 ? 1:-1 ) * (NEED_ACCUM_ANGEL);//SH_Varia.Dial_ADDAngle;//要移动的角度
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
		Dial_FlagState = 1;
	}
	else if(Dial_FlagState == 1 && (fabs(RC_Ctl.key.v - 1024.0) < 1.0f))
	{
		Dial_FlagState = 0;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	}
	
	return PID_Triple_Calculate(&PID_TSD.Ex_PID,
                           &PID_TSD.Min_PID,
                           &PID_TSD.In_PID,
                           SH_Varia.Dial_TAngle,
                           SH_Varia.Dial_AcumulateAngle,
                           CSpeed,
                           CCurrent,
                           20.0f);
	
	//return  PID_Double_Caculate(&PID_SD.In_PID,&PID_SD.Ex_PID,SH_Varia.Dial_TAngle,CSpeed,SH_Varia.Dial_AcumulateAngle,1.0f);
}


void Shooting_Friction(SF_Mood_e SF_Mood)
{
	if(SF_Mood == SH_NOMAL_MOOD)
	{
		Update_Friction_Variable();
		float CCurent_L = motor_chassis[FRICTION_L].given_current;
		float CCurent_R = motor_chassis[FRICTION_R].given_current;
		static float Out_L = 0;
		static float Out_R = 0;
	
		Out_L = PID_Double_Caculate(&PID_SF_L.In_PID,&PID_SF_L.Ex_PID,-(SH_Varia.Friction_TSpeed),CCurent_L,SH_Varia.Friction_CLSpeed,1.0f);
		Out_R = PID_Double_Caculate(&PID_SF_R.In_PID,&PID_SF_R.Ex_PID,SH_Varia.Friction_TSpeed,CCurent_R,SH_Varia.Friction_CRSpeed,1.0f);

		CAN_cmd_chassis(Out_R,Out_L,0,0);
	}
	else if(SF_Mood == SH_STOP_MOOD)
	{
		SH_Varia.Friction_TSpeed = 0;
		SH_Varia.Friction_CLSpeed = motor_chassis[FRICTION_L].speed_rpm;
		SH_Varia.Friction_CRSpeed = motor_chassis[FRICTION_R].speed_rpm;

		float CCurent_L = motor_chassis[FRICTION_L].given_current;
		float CCurent_R = motor_chassis[FRICTION_R].given_current;
		static float Out_L = 0;
		static float Out_R = 0;
		
		Out_L = PID_Double_Caculate(&PID_SF_L.In_PID,&PID_SF_L.Ex_PID,-(SH_Varia.Friction_TSpeed),CCurent_L,SH_Varia.Friction_CLSpeed,1.0f);
		Out_R = PID_Double_Caculate(&PID_SF_R.In_PID,&PID_SF_R.Ex_PID,SH_Varia.Friction_TSpeed,CCurent_R,SH_Varia.Friction_CRSpeed,1.0f);

		CAN_cmd_chassis(Out_R,Out_L,0,0);
	}
	else if(SF_Mood == SH_CHONTINUE_MOOD)
	{
		SH_Varia.Friction_TSpeed = FRICTION_RANGE;
		SH_Varia.Friction_CLSpeed = motor_chassis[FRICTION_L].speed_rpm;
		SH_Varia.Friction_CRSpeed = motor_chassis[FRICTION_R].speed_rpm;

		float CCurent_L = motor_chassis[FRICTION_L].given_current;
		float CCurent_R = motor_chassis[FRICTION_R].given_current;
		static float Out_L = 0;
		static float Out_R = 0;
		
		Out_L = PID_Double_Caculate(&PID_SF_L.In_PID,&PID_SF_L.Ex_PID,-(SH_Varia.Friction_TSpeed),CCurent_L,SH_Varia.Friction_CLSpeed,1.0f);
		Out_R = PID_Double_Caculate(&PID_SF_R.In_PID,&PID_SF_R.Ex_PID,SH_Varia.Friction_TSpeed,CCurent_R,SH_Varia.Friction_CRSpeed,1.0f);

		CAN_cmd_chassis(Out_R,Out_L,0,0);
	}
}

void Shooting_Friction_Stop(void)
{
	SH_Varia.Friction_TSpeed = 0;
	SH_Varia.Friction_CLSpeed = motor_chassis[FRICTION_L].speed_rpm;
	SH_Varia.Friction_CRSpeed = motor_chassis[FRICTION_R].speed_rpm;

	float CCurent_L = motor_chassis[FRICTION_L].given_current;
	float CCurent_R = motor_chassis[FRICTION_R].given_current;
	static float Out_L = 0;
	static float Out_R = 0;
	
	Out_L = PID_Double_Caculate(&PID_SF_L.In_PID,&PID_SF_L.Ex_PID,-(SH_Varia.Friction_TSpeed),CCurent_L,SH_Varia.Friction_CLSpeed,1.0f);
	Out_R = PID_Double_Caculate(&PID_SF_R.In_PID,&PID_SF_R.Ex_PID,SH_Varia.Friction_TSpeed,CCurent_R,SH_Varia.Friction_CRSpeed,1.0f);

	CAN_cmd_chassis(Out_R,Out_L,0,0);
}

//* 发射结构礼炮部分，返回PID计算后的输出值 *//
float Shooting_Salute(void)
{
	Update_Salute_Variable();//更新拨盘变量
	
	static uint8_t Salute_FlagState = 0;//发射机构状态
	static uint8_t TimeCount = 0;
	float CSSpeed = motor_chassis[7].speed_rpm;//更新电机当前速度
	
	if(TimeCount < 10)
	{
		SH_Varia.Salute_TAngle = SH_Varia.Salute_CAngle;
		SH_Varia.Salute_AcumulateAngle = SH_Varia.Salute_CAngle;
		TimeCount++;
	}
	else if(TimeCount >= 10)
	{
		if(RC_Ctl.rc.s2 == RC_LEVER_DOWN)
		{
			if((fabs(RC_Ctl.key.v - 1024.0) > 1.0f) && (Salute_FlagState == 0))
			{
				SH_Varia.Salute_TAngle = SH_Varia.Salute_TAngle + (SH_Varia.Salute_IsF == 1 ? -1:1 ) * (60.0f);//SH_Varia.Dial_ADDAngle;//要移动的角度
				Salute_FlagState = 1;
			}
			else if(Salute_FlagState == 1 && (fabs(RC_Ctl.key.v - 1024.0) < 1.0f))
			{
				Salute_FlagState = 0;
			}
			
			SH_Varia.Salute_TAngle = MyMath_Limit_Float(SH_Varia.Salute_TAngle,-180.0f,180.0f,1);
			SH_Varia.Salute_AcumulateAngle = MyMath_Limit_Float(SH_Varia.Salute_AcumulateAngle,-180.0f,180.0f,1);
		}
		else
		{
			SH_Varia.Salute_TAngle = SH_Varia.Salute_CAngle;
			SH_Varia.Salute_AcumulateAngle = SH_Varia.Salute_CAngle;
		}
	}
	
	return  PID_Double_CycleAngle(&PID_SS.In_PID,&PID_SS.Ex_PID,SH_Varia.Salute_TAngle,CSSpeed,SH_Varia.Salute_AcumulateAngle,1.0f);
}

//** 本地函数 **//
//* 拨盘 *//
/* 更新拨盘相关变量 */
static float Update_Dial_Variable(void)
{
	SH_Varia.Dial_ADDAngle = MyMath_Map_Range((RC_Ctl.key.v - 1024),-RC_KV_RANGE,RC_KV_RANGE,-10.0,10.0);
	SH_Varia.Dial_CAngle = motor_chassis[6].Degree_Angle;
	SH_Varia.Dial_AcumulateAngle = get_accumulated_angle(SH_Varia.Dial_CAngle);
	SH_Varia.Dial_IsF = ((RC_Ctl.key.v - 1024) >= 0) ? 1:0;
	
}
/* 更新彩弹发射 */
static float Update_Salute_Variable(void)
{
	SH_Varia.Salute_ADDAngle = MyMath_Map_Range((RC_Ctl.key.v - 1024),-RC_KV_RANGE,RC_KV_RANGE,-10.0,10.0);
	SH_Varia.Salute_CAngle = motor_chassis[7].Degree_Angle;
	SH_Varia.Salute_AcumulateAngle = get_accumulated_angle(SH_Varia.Salute_CAngle);
	SH_Varia.Salute_IsF = ((RC_Ctl.key.v - 1024) >= 0) ? 1:0;
}	
//* 更新摩擦轮相关变量 *//
static float Update_Friction_Variable(void)
{
	//SH_Varia.Friction_TSpeed = MyMath_Map_Range((RC_Ctl.key.v - 1024),-RC_KV_RANGE,RC_KV_RANGE,-FRICTION_RANGE,FRICTION_RANGE);
	
	SH_Varia.Friction_TSpeed = (RC_Ctl.key.v - 1024) > 0 ? 16384:0;
	SH_Varia.Friction_CLSpeed = motor_chassis[FRICTION_L].speed_rpm;
	SH_Varia.Friction_CRSpeed = motor_chassis[FRICTION_R].speed_rpm;
}



