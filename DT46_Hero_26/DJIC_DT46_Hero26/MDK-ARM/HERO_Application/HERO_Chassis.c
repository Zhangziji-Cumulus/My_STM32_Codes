#include "HERO_Chassis.h"


//获取四个麦轮的数据
void Chassis_Get_Data(Mecanum_Data_t* pMD,Wheels_Data_t* wheels)
{
	pMD->FL = wheels->W_FL;
	pMD->FR = wheels->W_FR;
	pMD->BL = wheels->W_BL;
	pMD->BR = wheels->W_BR;
}

//每个麦轮速度解算
void Chassis_Mecanum_Calc(Mecanum_Data_t* pMD)
{
	pMD->FL.T_Speed = -((pMD->Target.Xv) - (pMD->Target.Yv) + (pMD->Target.Rv));
	pMD->FR.T_Speed =  ((pMD->Target.Xv) + (pMD->Target.Yv) - (pMD->Target.Rv));
	pMD->BL.T_Speed =  ((pMD->Target.Xv) - (pMD->Target.Yv) - (pMD->Target.Rv));
	pMD->BR.T_Speed = -((pMD->Target.Xv) + (pMD->Target.Yv) + (pMD->Target.Rv));
}

//PID闭环返回输出值
void Chassis_PID_Caculate(Mecanum_Data_t* pMD ,Chassis_PID_t* PID,float MError)
{
	pMD->FL.Out_vel = PID_Double_Calculate(PID->In_Current,
																				 PID->Ex_Speed,
																				 pMD->FL.T_Speed,
																				 pMD->FL.C_Curr, 
																				 pMD->FL.C_Speed, 
																				 MError);
	
	pMD->FR.Out_vel = PID_Double_Calculate(PID->In_Current,
																				 PID->Ex_Speed,
																				 pMD->FR.T_Speed,
																				 pMD->FR.C_Curr, 
																				 pMD->FR.C_Speed, 
																				 MError);

	pMD->BL.Out_vel = PID_Double_Calculate(PID->In_Current,
																				 PID->Ex_Speed,
																				 pMD->BL.T_Speed,
																				 pMD->BL.C_Curr, 
																				 pMD->BL.C_Speed, 
																				 MError);
	
	pMD->BR.Out_vel = PID_Double_Calculate(PID->In_Current,
																				 PID->Ex_Speed,
																				 pMD->BR.T_Speed,
																				 pMD->BR.C_Curr, 
																				 pMD->BR.C_Speed, 
																				 MError);
}
