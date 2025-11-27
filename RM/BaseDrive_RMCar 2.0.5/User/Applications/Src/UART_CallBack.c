#include "UART_CallBack.h"



void MyUART_Init(void)
{
	//PID_USARTInit();
	Remote_ControlInit();//Init RemoteControl's UART
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) 
    {
		//PIDChangeByUSART_CallBack();
    }
    else if (huart == &huart3)
    {
		ReMote_Control_CallBack();
    }
}