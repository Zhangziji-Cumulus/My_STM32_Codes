// #include "AutoAim.h"

// #if(AutoAim_IFOPEN)

// static AutoAim_Instance_t  AutoAim_Instance;//自瞄实例
// static AutoAim_Ctrl_t AutoAim_Ctrl;//自瞄控制量
// //** #################################################################################################### **//
// //** ========================================= 对外函数 ================================================= **//
// //** #################################################################################################### **//

// //** ================================================================================ **//
// //** ================================== 融合算法 ==================================== **//
// //** ================================================================================ **//

// /**
//   * @brief  纯整数 int16_t 自瞄+手动融合函数（无浮点、最稳）
//   * @param  manual: 手动控制值 int16_t
//   * @param  auto_val: 自瞄控制值 int16_t
//   * @param  aim_valid: 自瞄是否有效（1=有效 0=无效）→ 你自己控制！
//   * @param  min_out: 输出最小值 int16_t
//   * @param  max_out: 输出最大值 int16_t
//   * @retval 最终输出 int16_t
//   */
// int16_t AutoAim_WeightFusion_Int16(int16_t manual, int16_t auto_val, uint8_t aim_valid, int16_t min_out, int16_t max_out)
// {
//     int32_t output;  // 用32位临时计算，防止溢出

//     // 自瞄无效 → 纯手动
//     if (aim_valid == 0)
//     {
//         output = manual;
//     }
//     // 自瞄有效 → 整数加权融合
//     else
//     {
//         output =  ((int32_t)auto_val * AUTOAIM_WEIGHT_AUTO + 
//                    (int32_t)manual  * AUTOAIM_WEIGHT_MANUAL) / 100;
//     }

//     // 输出限幅（固定范围，绝对不超界）
//     if (output > max_out) output = max_out;
//     if (output < min_out) output = min_out;

//     return (int16_t)output;
// }

// /**
//   * @brief  浮点型 自瞄+手动融合函数（顺滑无抖动）
//   * @param  manual: 手动控制值 float
//   * @param  auto_val: 自瞄控制值 float
//   * @param  aim_valid: 自瞄是否有效（1=有效 0=无效）
//   * @param  min_out: 输出最小值 float
//   * @param  max_out: 输出最大值 float
//   * @retval 最终输出 float
//   */
// float AutoAim_WeightFusion_Float(float manual, float auto_val, uint8_t aim_valid, float min_out, float max_out)
// {
//     float output;  // 浮点临时计算

//     // 自瞄无效 → 纯手动
//     if (aim_valid == 0)
//     {
//         output = manual;
//     }
//     // 自瞄有效 → 浮点加权融合（不需要除100以外的强转，更顺滑）
//     else
//     {
//         output =  auto_val * AUTOAIM_WEIGHT_AUTO + 
//                   manual  * AUTOAIM_WEIGHT_MANUAL;
//         // 权重和为100时，直接除以100
//         output /= 100.0f;
//     }

//     // 输出限幅（绝对不超界）
//     if (output > max_out)
//         output = max_out;
//     if (output < min_out)
//         output = min_out;

//     return output;
// }

// //** ================================================================================ **//
// //** ============================= 获取自瞄控制量 ==================================== **//
// //** ================================================================================ **//

// const AutoAim_Ctrl_t* AutoAim_Ctrl_Get_point(void)
// {
//     return &AutoAim_Ctrl;
// }

// //** #################################################################################################### **//
// //** ======================================= 上下位机通信 ================================================ **//
// //** #################################################################################################### **//

// //** ------------------------------------------------------------ **//
// //** ======================== 初始化通信 ======================== **//
// //** ------------------------------------------------------------ **//
// void AutoAim_Init(void)
// {
//     // 开启串口空闲中断
//     __HAL_UART_ENABLE_IT(&AUTO_USART_HANDLE, UART_IT_IDLE);

//     // 启动DMA接收，数据存入 Rx_Buff
//     HAL_UART_Receive_DMA(&AUTO_USART_HANDLE, AutoAim_Instance.Rx_Buff, sizeof(AutoAim_Rx_t));

// 	AutoAim_Instance.Tx_Done = 1;
	
//     AutoAim_Instance.Tx.Frame_head = AUTO_USART_HEADER;
//     AutoAim_Instance.Tx.Enemy_Color = AUTOAIM_ENEMY_COLOR;  
    
// }

// //** ================================================================================ **//
// //** =============================== 发送给上位机 ==================================== **//
// //** ================================================================================ **//

// //** ------------------------------------------------------------ **//
// //** ====================== 更新要发送的数据 ====================== **//
// //** ------------------------------------------------------------ **//
// void AutoAim_UpdateTx(void)
// {
//     //获取陀螺仪数据
//     AutoAim_Instance.MCUData.INS_angle =  IMU_Get_point();
//     AutoAim_Instance.Tx.IMU_Roll = AutoAim_Instance.MCUData.INS_angle[IMU_INDEX_ROLL];
//     AutoAim_Instance.Tx.IMU_Pitch = AutoAim_Instance.MCUData.INS_angle[IMU_INDEX_PITCH];
//     AutoAim_Instance.Tx.IMU_Yaw = AutoAim_Instance.MCUData.INS_angle[IMU_INDEX_YAW];

//     AutoAim_Instance.Tx.Match = 999;
// }

// //** ------------------------------------------------------------ **//
// //** ========================== 发送函数 ========================= **//
// //** ------------------------------------------------------------ **//
// void AutoAim_SendData(void)
// {
//     if (AutoAim_Instance.Tx_Done == 0)
//         return;

//     AutoAim_Instance.Tx_Done = 0;

//     // 直接发送结构体（不需要缓冲区拷贝）
//     HAL_UART_Transmit_DMA(&AUTO_USART_HANDLE, (uint8_t*)&AutoAim_Instance.Tx, sizeof(AutoAim_Tx_t));
// }

// // DMA发送完成回调
// void AutoAim_TxCpltCallback(void)
// {
//     AutoAim_Instance.Tx_Done = 1;
// }

// //** ================================================================================ **//
// //** ============================= 从上位机接受并解析 ================================ **//
// //** ================================================================================ **//

// //** ------------------------------------------------------------ **//
// //** ======================== 接收解析函数 ======================= **//
// //** ------------------------------------------------------------ **//
// void AutoAim_ReceiveProcess(void)
// {
//     AutoAim_Rx_t rx_buf;
//     // 拷贝DMA数据到结构体
//     memcpy(&rx_buf, AutoAim_Instance.Rx_Buff, sizeof(AutoAim_Rx_t));

//     // 1. 帧头校验
//     if(rx_buf.Frame_head != 0x5A)
//         return;
// 		AutoAim_Instance.Rx =rx_buf;

//     //更新自瞄的控制量
//     AutoAim_Ctrl.Yaw = AutoAim_Instance.Rx.Yaw; 
//     AutoAim_Ctrl.Pitch = AutoAim_Instance.Rx.Pitch;
//     AutoAim_Ctrl.FireOK = AutoAim_Instance.Rx.Fire;
// }

// //** ------------------------------------------------------------ **//
// //** ======================== 中断接收函数 ======================= **//
// //** ------------------------------------------------------------ **//

// /**
//  * @brief AutoAim_UART_IRQHandler中断处理函数
//  * @note 需在 USARTx_IRQHandler 中调用
//  */
// void AutoAim_UART_IRQHandler(void)
// {
//     HAL_UART_IRQHandler(&AUTO_USART_HANDLE);

//     // 空闲中断 = 一帧数据接收完成
//     if (__HAL_UART_GET_FLAG(&AUTO_USART_HANDLE, UART_FLAG_IDLE) != RESET)
//     {
//         __HAL_UART_CLEAR_IDLEFLAG(&AUTO_USART_HANDLE);

//         // 停止DMA
//         HAL_UART_DMAStop(&AUTO_USART_HANDLE);

//         // 解析数据
//         AutoAim_ReceiveProcess();

//         // 重新开启DMA接收
//         HAL_UART_Receive_DMA(&AUTO_USART_HANDLE, AutoAim_Instance.Rx_Buff, sizeof(AutoAim_Rx_t));
//     }
// }

// #endif


#include "AutoAim.h"

#if(AUTOAIM_IFOPEN)

static AutoAim_Instance_t  AutoAim_Instance;//自瞄实例
static AutoAim_Ctrl_t AutoAim_Ctrl;//自瞄控制量

//** ================================================================================ **//
//** ================================== 融合算法 ==================================== **//
//** ================================================================================ **//
int16_t AutoAim_WeightFusion_Int16(int16_t manual, int16_t auto_val, uint8_t aim_valid, int16_t min_out, int16_t max_out)
{
    int32_t output;

    if (aim_valid == 0)
    {
        output = manual;
    }
    else
    {
        output =  ((int32_t)auto_val * AUTOAIM_WEIGHT_AUTO +
                   (int32_t)manual  * AUTOAIM_WEIGHT_MANUAL) / 100;
    }

    if (output > max_out) output = max_out;
    if (output < min_out) output = min_out;

    return (int16_t)output;
}

float AutoAim_WeightFusion_Float(float manual, float auto_val, uint8_t aim_valid, float min_out, float max_out)
{
    float output;

    if (aim_valid == 0)
    {
        output = manual;
    }
    else
    {
        output =  auto_val * AUTOAIM_WEIGHT_AUTO +
                  manual  * AUTOAIM_WEIGHT_MANUAL;
        output /= 100.0f;
    }

    if (output > max_out)
        output = max_out;
    if (output < min_out)
        output = min_out;

    return output;
}

const AutoAim_Ctrl_t* AutoAim_Ctrl_Get_point(void)
{
    return &AutoAim_Ctrl;
}

//** ------------------------------------------------------------ **//
//** ======================== 初始化通信 ======================== **//
//** ------------------------------------------------------------ **//
void AutoAim_Init(void)
{
    __HAL_UART_ENABLE_IT(&AUTO_USART_HANDLE, UART_IT_IDLE);

    AutoAim_Instance.Rx_ActiveBuf = 0;
    // DMA循环模式绑定第一个缓冲，硬件自动循环切换
    HAL_UART_Receive_DMA(&AUTO_USART_HANDLE, (uint8_t *)&AutoAim_Instance.Rx_Buf[AutoAim_Instance.Rx_ActiveBuf], sizeof(AutoAim_Rx_t));

    AutoAim_Instance.Tx_Done = 1;
    AutoAim_Instance.Tx.Frame_head = AUTO_USART_HEADER;
    AutoAim_Instance.Tx.Enemy_Color = AUTOAIM_ENEMY_COLOR;
}

//** ====================== 更新要发送的数据 ====================== **//
void AutoAim_UpdateTx(void)
{
    AutoAim_Instance.MCUData.INS_angle =  IMU_Get_point();
    AutoAim_Instance.Tx.IMU_Roll = AutoAim_Instance.MCUData.INS_angle[IMU_INDEX_ROLL];
    AutoAim_Instance.Tx.IMU_Pitch = AutoAim_Instance.MCUData.INS_angle[IMU_INDEX_PITCH];
    AutoAim_Instance.Tx.IMU_Yaw = AutoAim_Instance.MCUData.INS_angle[IMU_INDEX_YAW];

    AutoAim_Instance.Tx.Match = 999;
}

//** ===================== 循环模式发送函数 ====================== **//
void AutoAim_SendData(void)
{
    HAL_UART_Transmit_DMA(&AUTO_USART_HANDLE, (uint8_t*)&AutoAim_Instance.Tx, sizeof(AutoAim_Tx_t));
}

//** ------------------------------------------------------------ **//
//** ======================== 接收解析函数 ======================= **//
//** ------------------------------------------------------------ **//
void ResetMatch(void)
{
    // 静态变量只会初始化一次
    static uint32_t last_change_tick = 0;  // 记录Match最后一次变化的系统时间
    static uint8_t  last_Match = 0;        // 上一帧的Match值

    uint8_t now_Match = AutoAim_Instance.Rx.Match;

    // ====================== 1. Match 发生变化 ======================
    if (now_Match != last_Match)
    {
        last_Match = now_Match;           // 更新旧值
        last_change_tick = HAL_GetTick(); // 更新【最后变化时间】
    }
    // ====================== 2. Match 长时间没变化 ======================
    else
    {
        // 判断：距离上一次变化 超过 1500ms（1.5秒）
        if (HAL_GetTick() - last_change_tick >= 1500)
        {
            AutoAim_Instance.Rx.Match = 0; // 超时重置Match
            // 这里不清除时间戳，避免反复触发
        }
    }
}

void AutoAim_ReceiveProcess(void)
{
    AutoAim_Rx_t *rx_buf = &AutoAim_Instance.Rx_ParseBuf;

    // 帧头校验
    if(rx_buf->Frame_head != AUTO_USART_HEADER)
        return;
    AutoAim_Instance.Rx = *rx_buf;
}

void AutoAim_UpdateRx(void)
{
    ResetMatch();

    //在线状态暂时设置为0
    if(AutoAim_Instance.Rx.Match != 0)
    {
        AutoAim_Ctrl.Yaw = AutoAim_Instance.Rx.Yaw;
        AutoAim_Ctrl.Pitch = AutoAim_Instance.Rx.Pitch;
        AutoAim_Ctrl.FireOK = AutoAim_Instance.Rx.Fire;
        AutoAim_Ctrl.IsOnline = 1;
    }
    else
    {
        AutoAim_Ctrl.Yaw = 0;
        AutoAim_Ctrl.Pitch = 0;
        AutoAim_Ctrl.FireOK = 0;
        AutoAim_Ctrl.IsOnline = 0;
    }
}

//** ------------------------------------------------------------ **//
//** ======================== 中断接收函数 ======================= **//
//** ------------------------------------------------------------ **//
void AutoAim_UART_IRQHandler(void)
{
    HAL_UART_IRQHandler(&AUTO_USART_HANDLE);

    if (__HAL_UART_GET_FLAG(&AUTO_USART_HANDLE, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&AUTO_USART_HANDLE);

        // 1. 把当前DMA写完的整帧拷贝到解析缓存（DMA此时在另一个缓冲继续存数据，不会被覆写）
        memcpy(&AutoAim_Instance.Rx_ParseBuf, &AutoAim_Instance.Rx_Buf[AutoAim_Instance.Rx_ActiveBuf], sizeof(AutoAim_Rx_t));

        // 2. 切换DMA目标缓冲，硬件自动切换，不再手动启停DMA
        AutoAim_Instance.Rx_ActiveBuf ^= 1U;
        HAL_UART_Receive_DMA(&AUTO_USART_HANDLE, (uint8_t *)&AutoAim_Instance.Rx_Buf[AutoAim_Instance.Rx_ActiveBuf], sizeof(AutoAim_Rx_t));

        // 3. 解析锁定好的数据
        AutoAim_ReceiveProcess();
    }
}

#endif