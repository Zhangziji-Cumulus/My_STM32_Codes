#include <stdint.h>
#include "PID.h"
#include "math.h"

//** 对外函数 **//


/**
 * @brief  初始化PID控制器
 * @param  pid: PID结构体指针
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @param  output_min: 输出最小值
 * @param  output_max: 输出最大值
 * @param  integral_min: 积分项最小值（建议与输出限幅匹配）
 * @param  integral_max: 积分项最大值（建议与输出限幅匹配）
 */
void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd,
              float output_min, float output_max,
              float integral_min, float integral_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->error_sum = 0.0f;

    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
}

/**
 * @brief  PID计算函数（位置式PID，带积分限幅和输出限幅）
 * @param  pid: PID结构体指针
 * @param  current_val: 当前测量值
 * @param  target_val: 目标值
 * @return 计算后的输出值（已限幅）
 */
float PID_Calculate(PID_HandleTypeDef *pid, float current_val, float target_val) {
    // 更新当前值和目标值
    pid->current = current_val;
    pid->target = target_val;

    // 计算当前误差
    pid->error = pid->target - pid->current;

    // 1. 比例项计算
    float p_out = pid->kp * pid->error;

    // 2. 积分项计算（带积分限幅，防止积分饱和）
    pid->error_sum += pid->ki * pid->error;
    // 积分限幅（限制误差累计值在合理范围）
    if (pid->error_sum > pid->integral_max) {
        pid->error_sum = pid->integral_max;
    } else if (pid->error_sum < pid->integral_min) {
        pid->error_sum = pid->integral_min;
    }
    float i_out = pid->error_sum;

    // 3. 微分项计算（基于当前误差与上一次误差的差值）
    float d_out = pid->kd * (pid->error - pid->last_error);

    // 保存当前误差，用于下一次微分计算
    pid->last_error = pid->error;

    // 总输出 = 比例项 + 积分项 + 微分项
    float total_out = p_out + i_out + d_out;

    // 输出限幅（防止执行器超出物理范围）
    if (total_out > pid->output_max) {
        total_out = pid->output_max;
    } else if (total_out < pid->output_min) {
        total_out = pid->output_min;
    }

    return total_out;
}

/* 双环PID控制计算函数 */
float PID_Double_Caculate(PID_HandleTypeDef* PID_In,PID_HandleTypeDef* PID_Ex,float Tartget,float Current_In,float Current_Ex,float MError)
{
	float PID_InOutput;
	float PID_ExOutput;
	
	PID_ExOutput = PID_Calculate(PID_Ex,Current_Ex,Tartget);
	
	

	
	if(fabs(Current_Ex - Tartget) <= MError)
	{
		PID_InOutput = 0;
	}
	else
	{
		PID_InOutput = PID_Calculate(PID_In,Current_In,PID_ExOutput);
	}
	
	PID_Ex->Output = PID_ExOutput;
	PID_In->Output = PID_InOutput;
	
	return PID_InOutput;
}

/**
 * @brief  三环PID控制计算函数（角度环 → 速度环 → 电流环）
 *         外环为角度控制，中环为速度控制，内环为电流控制
 * @param  PID_Angle: 角度环 PID 控制器（外环）
 * @param  PID_Speed: 速度环 PID 控制器（中环）
 * @param  PID_Current: 电流环 PID 控制器（内环）
 * @param  target_angle: 目标角度（rad 或 °）
 * @param  current_angle: 当前实际角度
 * @param  current_speed: 当前实际速度（由角度微分或编码器测得）
 * @param  current_current: 当前实际电流（q轴分量 Iq）
 * @param  max_angle_error: 角度误差阈值，小于该值才允许进入速度/电流控制（防止扰动）
 * @retval float: 电流环输出值（即最终控制量，可用于 SVPWM 调制）
 */
float PID_Triple_Calculate(PID_HandleTypeDef* PID_Angle,
                           PID_HandleTypeDef* PID_Speed,
                           PID_HandleTypeDef* PID_Current,
                           float target_angle,
                           float current_angle,
                           float current_speed,
                           float current_current,
                           float max_angle_error)
{
    float pid_angle_output = 0.0f;  // 角度环输出 → 目标速度
    float pid_speed_output = 0.0f;  // 速度环输出 → 目标电流
    float pid_current_output = 0.0f; // 电流环输出 → 最终控制量

    // =================== 【1】外环：角度环计算 ===================
    pid_angle_output = PID_Calculate(PID_Angle, current_angle, target_angle);

    // 保存角度环输出（便于调试）
    PID_Angle->Output = pid_angle_output;

    // =================== 【2】中环：速度环计算 ===================
    // 只有当角度误差较大时才激活速度和电流环（避免小误差抖动）
    if (fabsf(target_angle - current_angle) > max_angle_error)
    {
        pid_speed_output = PID_Calculate(PID_Speed, current_speed, pid_angle_output);
    }
    else
    {
        // 角度接近目标时，停止速度输出（可选：也可保持 hold torque）
        pid_speed_output = 0.0f;
        // 注意：若需零速悬停，应设置 speed_ref = 0 而非跳过控制
    }

    // 保存速度环输出
    PID_Speed->Output = pid_speed_output;

    // =================== 【3】内环：电流环计算 ===================
    pid_current_output = PID_Calculate(PID_Current, current_current, pid_speed_output);

    // 保存电流环输出
    PID_Current->Output = pid_current_output;

    // =================== 【4】返回最终控制输出 ===================
    return pid_current_output;
}

// 计算360°旋转场景下的最近角度误差（核心函数）
// 输入：当前角度current（°）、目标角度target（°），范围需预先确保在[0, 360)
// 输出：最近方向的误差（范围[-180, 180]），正为顺时针，负为逆时针
float PID_CalcNearestAngleError(float current, float target) {
    float error = target - current;
    
    // 标准化误差到[-180, 180)，确保走最近路径
    error = fmod(error, 360.0f);  // 先取模到[-360, 360)
    if (error > 180.0f) {
        error -= 360.0f;  // 大于180°时，反向走更短路径
    } else if (error < -180.0f) {
        error += 360.0f;  // 小于-180°时，反向走更短路径
    }
    return error;
}

// 云台PID控制主函数（需周期性调用，建议固定周期）
// 输入：pid结构体指针、当前角度current（°）、目标角度target（°）
// 输出：计算后的控制量（已限幅）
float PID_Calculate_CycleAngle(PID_HandleTypeDef *pid, float current, float target) {
    // 1. 角度预处理：确保输入在[0, 360)范围（防止超范围导致误差计算错误）
    current = fmod(current, 360.0f);
    if (current < 0.0f) current += 360.0f;  // 负角度转正
    target = fmod(target, 360.0f);
    if (target < 0.0f) target += 360.0f;
    
    // 2. 更新当前状态与目标值
    pid->current = current;
    pid->target = target;
    
    // 3. 计算最近角度误差（核心：确保旋转路径最短）
    pid->error = PID_CalcNearestAngleError(current, target);
    
    // 4. 积分项计算（带积分限幅，防止积分饱和）
    pid->error_sum += pid->error;
    // 积分限幅（限制累计误差范围，避免超调）
    if (pid->error_sum > pid->integral_max) {
        pid->error_sum = pid->integral_max;
    } else if (pid->error_sum < pid->integral_min) {
        pid->error_sum = pid->integral_min;
    }
    
    // 5. 微分项计算（基于当前误差与上一次误差的变化）
    float diff = pid->error - pid->last_error;  // 误差变化率（假设调用周期固定）
    
    // 6. 计算PID输出（比例+积分+微分）
    float output = pid->kp * pid->error + 
                   pid->ki * pid->error_sum + 
                   pid->kd * diff;
    
    // 7. 输出限幅（防止执行器饱和，如电机PWM超出范围）
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }
    
    // 8. 保存当前误差为下一次的历史误差
    pid->last_error = pid->error;
    
    // 9. 更新输出并返回
    pid->Output = output;
    return output;
}
//* PID 循环角度双环控制函数 *//
float PID_Double_CycleAngle(PID_HandleTypeDef* PID_In,PID_HandleTypeDef* PID_Ex,float Tartget,float Current_In,float Current_Ex,float MError)
{

	float PID_InOutput;
	float PID_ExOutput;
	
	PID_ExOutput = PID_Calculate_CycleAngle(PID_Ex,Current_Ex,Tartget);
	
	

	
	if(fabs(Current_Ex - Tartget) <= MError)
	{
		PID_InOutput = 0;
	}
	else
	{
		PID_InOutput = PID_Calculate(PID_In,Current_In,PID_ExOutput);
	}
	
	PID_Ex->Output = PID_ExOutput;
	PID_In->Output = PID_InOutput;
	
	return PID_InOutput;
}




///*
//	PID Change by USART Variables and Types
//*/


///* 定义接收缓冲区和标志 */

//#define MAX_BUFFER_SIZE 256
//uint8_t rxBuffer[MAX_BUFFER_SIZE];  // 接收缓冲区
//uint8_t rxComplete = 0;          // 接收完成标志
//uint16_t rxLength = 0;           // 接收长度

//uint8_t CharRxIndex;
//uint8_t StringRxIndex;
//uint8_t FloatRxValueCount;
//char CharBuF[3][64];
//uint8_t PIDRxFlag = 0;

//float FloatValue[3] = {0.0f};

//extern DMA_HandleTypeDef hdma_usart1_rx;    // USART1_RX的DMA句柄

///* 字符串转浮点数函数 */
//float stringToFloat(const char* str);

//void PID_USARTInit(void)
//{
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
//	HAL_UART_Receive_DMA(&huart1,rxBuffer,256);
//}

//void ForceDMACompletion(void)
//{
//	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
//    {
//        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
//        
//        // 停止DMA传输
//        HAL_UART_DMAStop(&huart1);
//        
//        // 计算实际接收到的数据长度
//        rxLength = MAX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
//        
//        if (rxLength > 0)
//        {
//            rxBuffer[rxLength] = '\0';  // 添加字符串结束符
//            rxComplete = 1;             // 标记接收完成
//        }
//        // 处理接收到的数据（可在主循环中检查rxComplete）
//        PIDChangeByUSART_CallBack();
//        // 重新启动DMA接收
//        HAL_UART_Receive_DMA(&huart1, rxBuffer, MAX_BUFFER_SIZE);
//    }
//}

//void PIDChangeByUSART_CallBack(void)
//{
//	for(int i = 0;i < rxLength;i++)
//	{
//		if(rxBuffer[i] == ' ')	
//		{
//			CharBuF[StringRxIndex][CharRxIndex] = '\0';
//			StringRxIndex++;
//			CharRxIndex = 0;
//		}
//		else if(rxBuffer[i] == '\n' || rxBuffer[i] == '\r')
//		{
//			CharBuF[StringRxIndex][CharRxIndex] = '\0';
//			CharRxIndex = 0;
//			StringRxIndex++;
//		}
//		else
//		{
//			CharBuF[StringRxIndex][CharRxIndex] = rxBuffer[i];
//			CharRxIndex++;
//		}
//	}
//	PIDRxFlag = 1;
//}
// 
///* 获取PID值 */
//void PID_ValueGet(PID_HandleTypeDef* pPID)
//{
//	if(PIDRxFlag == 1)
//	{
//		for (int i = 0; i <= StringRxIndex; i++) 
//		{
//          FloatValue[i] = stringToFloat(CharBuF[i]);
//        }
//	    pPID->Kp = FloatValue[0];
//	    pPID->Ki = FloatValue[1];
//	    pPID->Kd = FloatValue[2];
//		CharRxIndex = 0;
//		StringRxIndex = 0;
//		PIDRxFlag = 0;
//	}
//}

///* 字符串转浮点数函数 */
//float stringToFloat(const char* str) {
//    float value = 0.0f;
//    int sign = 1;
//    int decimal = 0;
//    float factor = 0.1f;
//    
//    /* 处理符号 */
//    if (*str == '-') {
//        sign = -1;
//        str++;
//    } else if (*str == '+') {
//        str++;
//    }
//    
//    /* 处理整数部分 */
//    while (*str >= '0' && *str <= '9') {
//        value = value * 10.0f + (*str - '0');
//        str++;
//    }
//    
//    /* 处理小数点 */
//    if (*str == '.') {
//        str++;
//        decimal = 1;
//    }
//    
//    /* 处理小数部分 */
//    while (decimal && *str >= '0' && *str <= '9') {
//        value += (*str - '0') * factor;
//        factor *= 0.1f;
//        str++;
//    }
//    
//    return value * sign;
//}

