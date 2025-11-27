#ifndef __CAN_BSP_H
#define __CAN_BSP_H

#include "main.h"

//**  宏定义 **//


//** 结构体定义 **//

/* CAN发送消息参数结构体 */
typedef struct {
    CAN_HandleTypeDef *hcan;        /* CAN外设句柄指针（如&hcan1, &hcan2） */
    uint32_t std_id;                /* 标准ID (11位，0~0x7FF) */
    uint32_t ext_id;                /* 扩展ID (29位，0~0x1FFFFFFF) */
    uint8_t is_ext_id;              /* 是否使用扩展ID (0=标准ID, 1=扩展ID) */
    uint8_t data_len;               /* 数据长度 (0~8) */
    uint8_t *data;                  /* 待发送的数据指针 */
} CAN_Send_Msg_Config_t;

/** CAN过滤器枚举和结构体 **/

/* CAN过滤器枚举 */
typedef enum {
    CAN_FRAME_STD,	//标准帧
    CAN_FRAME_EXT	//扩展帧
} CAN_FrameType;
/* CAN过滤器结构体 */
typedef struct {
    CAN_HandleTypeDef *hcan; //CAN1、2等
    CAN_FrameType frame_type;//帧类型
    uint32_t *accept_ids;    // 实际 CAN ID 列表（如 0x123 或 0x18FF20A1）
    uint8_t id_count;        // ID 数量
} CAN_FilterConfig;

//** 对外函数声明 **//
void My_CAN_Init(void);//在这个函数里初始化CAN和过滤器
HAL_StatusTypeDef CAN_SendMessage(CAN_Send_Msg_Config_t *msg_config);

#endif
