#include "Time.h"

extern uint8_t ProgramState;
extern uint8_t BOARD_ID_Int;
extern uint8_t BIM088_ReSetFlag;
extern RC_Ctl_t RC_Ctl;

void MyTIM6Callback(void)
{
	if(ProgramState)
	{
		float Shooting_Dial_PIDOut;
		
		if(BOARD_ID_Int == 1)
		{
			if(BIM088_ReSetFlag)
			{
				GimBal_Pitch_Motor_Set();
				
				if(RC_Ctl.rc.s2 == RC_LEVER_DOWN)
				{
					Shooting_Friction(SH_NOMAL_MOOD);
				}
				else if(RC_Ctl.rc.s2 == RC_LEVER_MID)
				{
					Shooting_Friction(SH_STOP_MOOD);
				}
				else if(RC_Ctl.rc.s2 == RC_LEVER_UP)
				{
					Shooting_Friction(SH_CHONTINUE_MOOD);
				}
			}
		}
		else if(BOARD_ID_Int == 2)
		{
			if(BIM088_ReSetFlag)
			{
				Chassis_Motor_Set();
			}
			
			if(RC_Ctl.rc.s2 == RC_LEVER_DOWN || RC_Ctl.rc.s2 == RC_LEVER_UP)
			{
				Shooting_Dial_PIDOut = Shooting_Dial();
			}
			else
			{
				Shooting_Dial_PIDOut = 0;
			}
			GimBal_Yaw_Motor_Set(Shooting_Dial_PIDOut,0);
		}
	}
	
	//** LED **//
	static uint16_t Count = 0;
	Count++;
	if(Count>1000)
	{
		HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_10);
		
		Count = 0;
	}	
}
