#ifndef __BSP_INIT_H__
#define __BSP_INIT_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "bsp_CAN.h"
#include "bsp_SBUS.h"

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化底层驱动模块
 */
void BSP_Init(void);

#endif /* __BSP_INIT_H__ */
