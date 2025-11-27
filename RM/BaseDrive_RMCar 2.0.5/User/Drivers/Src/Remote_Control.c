#include "Remote_Control.h"

/*

实验室传承定义
定义右拨扭：2 无力，3 遥控，1 键鼠
定义左拨扭：2 正常，3 小陀螺，1自瞄

*/

RC_Ctl_t RC_Ctl;
uint8_t sbus_rx_buffer[18];

void Remote_ControlInit(void)
{
  HAL_UART_Receive_DMA(&huart3,sbus_rx_buffer,18);
}

void ReMote_Control_CallBack(void)
{
	RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff;          
	RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;       
	RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;          
	RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;           
	RC_Ctl.rc.s1  = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;                           
	RC_Ctl.rc.s2  = ((sbus_rx_buffer[5] >> 4)& 0x0003);    

	RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);                    //!< Mouse X axis        
	RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);                    //!< Mouse Y axis      
	RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);                  //!< Mouse Z axis         
	RC_Ctl.mouse.press_l = sbus_rx_buffer[12];                                        //!< Mouse Left Is Press      
	RC_Ctl.mouse.press_r = sbus_rx_buffer[13];                                        //!< Mouse Right Is Press 
	RC_Ctl.key.v = sbus_rx_buffer[16] | (sbus_rx_buffer[17] << 8);   			//!< KeyBoard value
				
	Dual_Board_Send();
}





