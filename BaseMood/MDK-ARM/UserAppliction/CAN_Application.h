#ifndef __CAN_APPLICATION_H
#define __CAN_APPLICATION_H

#include "main.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// 错误码枚举 
typedef enum {
    MOTOR_ERR_NONE                  = 0,//无异常
    MOTOR_ERR_OVER_VOLTAGE          = 2,//电调供电电压过高（仅开机自检）
    MOTOR_ERR_PHASE_NOT_CONNECTED   = 3,//电机三相线未接入
    MOTOR_ERR_SENSOR_DATA_LOST      = 4,//与电机相连的数据线中位置传感器数据丢失
    MOTOR_ERR_STALL                 = 6,//电机堵转
    MOTOR_ERR_CALIBRATION_FAILED    = 7 //电机校准失败
} MotorErrorCode_t;

// 定义电机反馈数据结构
typedef struct {
    uint16_t id;            // 电机ID (1-8)
    uint16_t angle_raw;     // 原始角度值 (0-8191)
    float angle_deg;        // 角度值 (0-360度)
    int16_t speed_rpm;      // 转速 (rpm)
    int16_t current_ma;     // 电流 (mA, 通常电调反馈单位为mA)
    MotorErrorCode_t error_code;     // 错误码	电机错误码：
    bool is_online;         // 在线标志位，用于检测电机是否掉线
} MotorFeedback_t;

void CAN_Parse_Motor_Feedback(uint32_t std_id, uint8_t* data);

#endif
