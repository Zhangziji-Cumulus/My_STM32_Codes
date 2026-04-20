#include "HERO_Chassis.h"

//每个麦轮速度解算
void Chassis_Mecanum_Calc(Mecanum_Data_t* pMD)
{
	pMD->FL.T_Speed = -((pMD->Target.Xv) - (pMD->Target.Yv) + (pMD->Target.Rv));
	pMD->FR.T_Speed =  ((pMD->Target.Xv) + (pMD->Target.Yv) - (pMD->Target.Rv));
	pMD->BL.T_Speed =  ((pMD->Target.Xv) - (pMD->Target.Yv) - (pMD->Target.Rv));
	pMD->BR.T_Speed = -((pMD->Target.Xv) + (pMD->Target.Yv) + (pMD->Target.Rv));
}
