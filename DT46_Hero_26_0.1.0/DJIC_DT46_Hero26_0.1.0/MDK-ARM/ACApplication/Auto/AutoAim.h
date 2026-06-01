#ifndef AUTOAIM_H_
#define AUTOAIM_H_

#include "A_MCommon.h"



// 1. 通信包必须 4 字节对齐（#pragma pack(push, 1) 是 1 字节对齐，最稳）
#pragma pack(push, 1)

// 接收数据：上位机 -> 下位机
typedef struct
{
    float Yaw;
    float Pitch;

    int8_t Fire;          // 开火
    int8_t reserved0;     // 对齐
    int8_t reserved1;     // 对齐
    int8_t reserved2;     // 对齐

    int32_t Match;        // 匹配标志

} AutoAim_Rx_t;         // 固定大小：20 字节

// 发送数据：下位机 -> 上位机
typedef struct
{
    uint8_t Frame_head;       //串口发送帧头

    uint8_t Enemy_Color;  // 敌方颜色,0=红，1=蓝
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t reserved2;

    float IMU_Roll;
    float IMU_Pitch;
    float IMU_Yaw;

    int32_t Match;        // 匹配标志

} AutoAim_Tx_t;         // 固定大小：28 字节

#pragma pack(pop)

typedef struct
{
    //下位机变量
    struct{

        const fp32* INS_angle;  //INS角度
        uint8_t  Self_Color;    //己方颜色

    }MCUData;

    // 通信接收
    AutoAim_Rx_t Rx;

    uint8_t Rx_Buff[sizeof(AutoAim_Rx_t)];

    // 通信发送
    AutoAim_Tx_t Tx;
    uint8_t Tx_Buff[sizeof(AutoAim_Tx_t)];

}AutoAim_Instance_t;

/* 纯整数 int16_t 自瞄+手动融合函数（无浮点、最稳）*/
int16_t AutoAim_WeightFusion(int16_t manual, int16_t auto_val, uint8_t aim_valid, int16_t min_out, int16_t max_out);

#endif // AUTOAIM_H_