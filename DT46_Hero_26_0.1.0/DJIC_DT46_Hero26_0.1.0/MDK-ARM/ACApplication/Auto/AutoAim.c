#include "AutoAim.h"

AutoAim_Instance_t  AutoAim_Instance;

//** #################################################################################################### **//
//** ========================================= 融合算法 ================================================= **//
//** #################################################################################################### **//

/**
  * @brief  纯整数 int16_t 自瞄+手动融合函数（无浮点、最稳）
  * @param  manual: 手动控制值 int16_t
  * @param  auto_val: 自瞄控制值 int16_t
  * @param  aim_valid: 自瞄是否有效（1=有效 0=无效）→ 你自己控制！
  * @param  min_out: 输出最小值 int16_t
  * @param  max_out: 输出最大值 int16_t
  * @retval 最终输出 int16_t
  */
int16_t AutoAim_WeightFusion(int16_t manual, int16_t auto_val, uint8_t aim_valid, int16_t min_out, int16_t max_out)
{
    int32_t output;  // 用32位临时计算，防止溢出

    // 自瞄无效 → 纯手动
    if (aim_valid == 0)
    {
        output = manual;
    }
    // 自瞄有效 → 整数加权融合
    else
    {
        output =  ((int32_t)auto_val * AUTOAIM_WEIGHT_AUTO + 
                   (int32_t)manual  * AUTOAIM_WEIGHT_MANUAL) / 100;
    }

    // 输出限幅（固定范围，绝对不超界）
    if (output > max_out) output = max_out;
    if (output < min_out) output = min_out;

    return (int16_t)output;
}

//** #################################################################################################### **//
//** ======================================= 上下位机通信 ================================================ **//
//** #################################################################################################### **//

//** ------------------------------------------------------------ **//
//** ======================== 初始化通信 ======================== **//
//** ------------------------------------------------------------ **//
void AutoAim_Init(void)
{
    // 开启串口空闲中断
    __HAL_UART_ENABLE_IT(&AUTO_USART_HANDLE, UART_IT_IDLE);

    // 启动DMA接收，数据存入 Rx_Buff
    HAL_UART_Receive_DMA(&AUTO_USART_HANDLE, AutoAim_Instance.Rx_Buff, sizeof(AutoAim_Rx_t));

    AutoAim_Instance.Tx.Frame_head = AUTO_USART_HEADER;
    AutoAim_Instance.Tx.Enemy_Color = AUTOAIM_ENEMY_COLOR;  
}

//** ================================================================================ **//
//** =============================== 发送给上位机 ==================================== **//
//** ================================================================================ **//

//** ------------------------------------------------------------ **//
//** ====================== 更新要发送的数据 ====================== **//
//** ------------------------------------------------------------ **//
void AutoAim_UpdateTx(void)
{
    //获取陀螺仪数据
    AutoAim_Instance.MCUData.INS_angle =  IMU_Get_point();
    AutoAim_Instance.Tx.IMU_Roll = AutoAim_Instance.MCUData.INS_angle[IMU_INDEX_ROLL];
    AutoAim_Instance.Tx.IMU_Pitch = AutoAim_Instance.MCUData.INS_angle[IMU_INDEX_PITCH];
    AutoAim_Instance.Tx.IMU_Yaw = AutoAim_Instance.MCUData.INS_angle[IMU_INDEX_YAW];

    AutoAim_Instance.Tx.Match = 999;
}

//** ------------------------------------------------------------ **//
//** ========================== 发送函数 ========================= **//
//** ------------------------------------------------------------ **//
void AutoAim_SendData(void)
{
    // 等待串口空闲
    if (HAL_UART_GetState(&AUTO_USART_HANDLE) != HAL_UART_STATE_READY)
        return;

    // 直接发送结构体（不需要缓冲区拷贝）
    HAL_UART_Transmit_DMA(&AUTO_USART_HANDLE, (uint8_t*)&AutoAim_Instance.Tx, sizeof(AutoAim_Tx_t));
}

//** ================================================================================ **//
//** ============================= 从上位机接受并解析 ================================ **//
//** ================================================================================ **//

//** ------------------------------------------------------------ **//
//** ======================== 接收解析函数 ======================= **//
//** ------------------------------------------------------------ **//
void AutoAim_Pocesss(void)
{

}

//** ------------------------------------------------------------ **//
//** ======================== 中断接收函数 ======================= **//
//** ------------------------------------------------------------ **//
void AutoAim_RxIT_FeedBack(void)
{
    HAL_UART_IRQHandler(&AUTO_USART_HANDLE);

    // 空闲中断 = 一帧数据接收完成
    if (__HAL_UART_GET_FLAG(&AUTO_USART_HANDLE, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&AUTO_USART_HANDLE);

        // 停止DMA
        HAL_UART_DMAStop(&AUTO_USART_HANDLE);

        // 解析数据
        AutoAim_ReceiveProcess();

        // 重新开启DMA接收
        HAL_UART_Receive_DMA(&AUTO_USART_HANDLE, AutoAim_Instance.Rx_Buff, sizeof(AutoAim_Rx_t));
    }
}

