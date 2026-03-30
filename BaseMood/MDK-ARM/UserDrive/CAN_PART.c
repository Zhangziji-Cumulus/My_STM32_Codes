#include "CAN_PART.h"

/* 包含必要的头文件，通常在 main.h 或 can.h 中已包含 */
#include "main.h" 

/**
  * @brief  自定义 CAN 初始化配置
  *         配置滤波器接收所有 ID，启动 CAN，开启中断
  * @param  hcan: CAN 句柄指针
  * @retval 无
  */
void MY_CAN_Init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef sFilterConfig;
    HAL_StatusTypeDef status;

    /* 1. 配置 CAN 滤波器 (接收所有 ID) */
    sFilterConfig.FilterBank = 0;                 // 使用滤波器组 0
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // 掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32 位位宽
    sFilterConfig.FilterIdHigh = 0x0000;          // ID 高 16 位
    sFilterConfig.FilterIdLow = 0x0000;           // ID 低 16 位
    sFilterConfig.FilterMaskIdHigh = 0x0000;      // 掩码高 16 位 (0 表示不检查，即接收所有)
    sFilterConfig.FilterMaskIdLow = 0x0000;       // 掩码低 16 位
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // 分配到 FIFO0
    sFilterConfig.FilterActivation = ENABLE;      // 激活滤波器
    sFilterConfig.SlaveStartFilterBank = 14;      // 如果是双 CAN，从机滤波器起始组，单机可忽略

    status = HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
    if (status != HAL_OK)
    {
        /* 滤波器配置错误处理 */
        Error_Handler(); 
    }

    /* 2. 启动 CAN 外设 */
    status = HAL_CAN_Start(hcan);
    if (status != HAL_OK)
    {
        /* 启动错误处理 */
        Error_Handler();
    }

    /* 3. 开启接收中断 (FIFO0 挂起中断) */
    status = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    if (status != HAL_OK)
    {
        /* 中断开启错误处理 */
        Error_Handler();
    }
}

/**
  * @brief  CAN FIFO0 消息挂起回调函数 (接收中断触发)
  * @param  hcan: CAN 句柄指针
  * @retval 无
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    HAL_StatusTypeDef status;

    /* 1. 获取接收到的消息 */
    status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

    if (status == HAL_OK)
    {

				CAN_Parse_Motor_Feedback(RxHeader.StdId,RxData);
           
    }    
}

