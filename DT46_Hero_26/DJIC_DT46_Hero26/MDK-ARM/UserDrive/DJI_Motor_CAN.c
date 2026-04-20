#include "DJI_Motor_CAN.h"
//#include "can.h"

/*
	3508使用C620电调支持ID 1-8，从0x200+ID开始，0x201/0x202/0x203/0x204/0x205/0x206/0x207/0x208
	控制电流值范围 -16384~0~16384，对应电调输出的转矩电流范围-20~0~20A
	！！但是！！M3508搭配C620电调额定电流是10A
	
	2006使用C610电调支持ID 1-8，从0x200+ID开始，0x201/0x202/0x203/0x204/0x205/0x206/0x207/0x208
	控制转矩电流值范围 -10000~0~10000，对应电调输出的转矩电流范围 -10~0~10A
	！！但是！！M2006搭配C610电调额定电流3A
	
	6020自带驱动支持 ID1-7，从0x204+ID开始，0x205/0x206/0x207/0x208/0x209/0x20A/0x20B
	控制电压值范围：-25000~0~25000
	控制电流值范围：-16384~0~16384, 对应最大转矩电流范围 -3A~0~3A
	
*/

//** 接受电机反馈数据 **//

//* 定义结构体 *//
DJI_MotorFeedback_t DJI_MFeedback[8];

//* 对外函数 *//
/**
  * @brief  解析电调反馈报文 (基于图片协议)
  * @param  std_id: CAN 标准帧 ID (例如 0x201)
  * @param  data: 8字节数据数组
  * @retval None
  */
void CAN_DJI_Motor_Feedback(uint32_t std_id, uint8_t* data)
{
    // 1. 检查 ID 是否在反馈范围内 (0x200 + ID)
    // 假设支持 ID 1 到 8，即 0x201 到 0x208
    if (std_id >= 0x201 && std_id <= 0x208) 
    {
        uint8_t index = std_id - 0x201; // 将 ID 映射到数组索引 0-7
        
        DJI_MFeedback[index].id = index + 1;
        DJI_MFeedback[index].is_online = true; // 收到数据，标记在线

        // 2. 解析角度 (DATA[0] 高8位, DATA[1] 低8位)
        // 范围 0-8191 对应 0-360度
        DJI_MFeedback[index].angle_raw = (uint16_t)((data[0] << 8) | data[1]);
        
        // 转换为角度值 (可选，方便计算)
        DJI_MFeedback[index].angle_deg = (float)DJI_MFeedback[index].angle_raw * 360.0f / 8192.0f;

        // 3. 解析转速 (DATA[2] 高8位, DATA[3] 低8位)
        // 单位 rpm
        DJI_MFeedback[index].speed_rpm = (int16_t)((data[2] << 8) | data[3]);

        // 4. 解析电流 (DATA[4] 高8位, DATA[5] 低8位)
        // 注意：图片未明确单位，但通常 DJI 电调反馈单位为 mA
        DJI_MFeedback[index].current_ma = (int16_t)((data[4] << 8) | data[5]);

        // 5. 解析错误码 (DATA[7])
        DJI_MFeedback[index].error_code = (DJI_MotorErrorCode_t)data[7];
        
        // DATA[6] 为空，忽略
    }
}



//** 控制电机函数 **//

//* 对外函数 *//

/* ================= 电调配置实例 ================= */

// 20A 版本电调配置 (协议 1: -20A~20A 对应 -16384~16384)
const ESC_Config_t ESC_C620_20A = {
    .name = "ESC_20A_Protocol",
    .max_current_amps = 20.0f,
    .max_raw_value = 16384
};

// 10A 版本电调配置 (协议 2: -10A~10A 对应 -10000~10000)
const ESC_Config_t ESC_C610_10A = {
    .name = "ESC_10A_Protocol",
    .max_current_amps = 10.0f,
    .max_raw_value = 10000
};

/* ================= 内部辅助函数 ================= */

// 16 位整数转大端模式 (高 8 位在前，低 8 位在后)
static void Int16_To_BigEndian(int16_t value, uint8_t* high, uint8_t* low) {
    uint16_t u_value = (uint16_t)value;
    *high = (u_value >> 8) & 0xFF;
    *low  = u_value & 0xFF;
}

// 核心换算：安培 → 原始值
static int16_t Amps_To_Raw(const ESC_Config_t* config, float amps) {
    if (config == NULL || config->max_current_amps <= 0) {
        return 0;
    }

    // 计算比例系数
    float ratio = (float)config->max_raw_value / config->max_current_amps;
    
    // 换算
    int32_t raw_val_32 = (int32_t)(amps * ratio);
    
    // 限幅保护
    if (raw_val_32 > config->max_raw_value) 
        raw_val_32 = config->max_raw_value;
    if (raw_val_32 < -config->max_raw_value) 
        raw_val_32 = -config->max_raw_value;
    if (raw_val_32 > 32767) raw_val_32 = 32767;
    if (raw_val_32 < -32768) raw_val_32 = -32768;

    return (int16_t)raw_val_32;
}

// 获取 CAN ID
static uint32_t Get_CAN_ID(uint8_t motor_start_id) {
    if (motor_start_id == 1) return ESC_CAN_ID_GROUP_1;
    if (motor_start_id == 5) return ESC_CAN_ID_GROUP_2;
    return 0;
}

/* ================= 底层发送 (STM32 HAL) ================= */

extern CAN_HandleTypeDef hcan1;

void CAN_DJI_SendSTD(uint32_t id, uint8_t* data) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    TxHeader.StdId = id;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    // 等待发送邮箱空闲
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox);
}

/* ================= 安培值控制函数 ================= */

/**
 * @brief 安培值控制单个电机
 * @param can指针句柄
 * @param config: 电调配置指针 (如 &ESC_CONFIG_20A)
 * @param motor_id: 电机 ID (1 ~ 8)
 * @param current_amps: 目标电流 (单位：安培 A)，如 5.5f 或 -3.0f
 */
void ESC_Control_Amps_Single(CAN_HandleTypeDef *hcan,const ESC_Config_t* config, uint8_t motor_id, float current_amps){
    uint8_t data[8] = 0;
    uint32_t can_id = 0;
    int data_index = 0;

    if (motor_id >= 1 && motor_id <= 4) {
        can_id = ESC_CAN_ID_GROUP_1;
        data_index = (motor_id - 1) * 2;
    } else if (motor_id >= 5 && motor_id <= 8) {
        can_id = ESC_CAN_ID_GROUP_2;
        data_index = (motor_id - 5) * 2;
    } else {
        return;
    }

    int16_t raw_val = Amps_To_Raw(config, current_amps);
    Int16_To_BigEndian(raw_val, &data[data_index], &data[data_index + 1]);
    CAN_Send_STD(hcan,can_id, data);
}

/**
 * @brief 安培值控制一组电机 (4个)
 * @param config: 电调配置指针
 * @param motor_start_id: 起始电机 ID (1 或 5)
 * @param currents_amps: 长度为 4 的浮点数组，对应 4 个电机的电流
 */
void ESC_Control_Amps_Group(CAN_HandleTypeDef *hcan,const ESC_Config_t* config, uint8_t motor_start_id, float currents_amps[4]) {
    uint8_t data[8];
    uint32_t can_id = Get_CAN_ID(motor_start_id);
    
    if (can_id == 0) return;

    for (int i = 0; i < 4; i++) {
        int16_t raw_val = Amps_To_Raw(config, currents_amps[i]);
        Int16_To_BigEndian(raw_val, &data[i*2], &data[i*2 + 1]);
    }

    CAN_Send_STD(hcan,can_id, data);
}

/**
 * @brief 安培值控制所有电机 (8个)
 * @param config: 电调配置指针
 * @param currents_amps: 长度为 8 的浮点数组，索引 0 对应电机 1
 */
void ESC_Control_Amps_All(CAN_HandleTypeDef *hcan,const ESC_Config_t* config, float currents_amps[8]) {
    uint8_t data[8];

    // 发送第一帧 (电机 1-4)
    for (int i = 0; i < 4; i++) {
        int16_t raw_val = Amps_To_Raw(config, currents_amps[i]);
        Int16_To_BigEndian(raw_val, &data[i*2], &data[i*2 + 1]);
    }
    CAN_DJI_SendSTD(ESC_CAN_ID_GROUP_1, data);

    // 发送第二帧 (电机 5-8)
    for (int i = 0; i < 4; i++) {
        int16_t raw_val = Amps_To_Raw(config, currents_amps[i + 4]);
        Int16_To_BigEndian(raw_val, &data[i*2], &data[i*2 + 1]);
    }
    CAN_Send_STD(hcan,ESC_CAN_ID_GROUP_2, data);
}

/* ================= 原始值控制函数 ================= */

/**
 * @brief 原始值控制单个电机
 * @param motor_id: 电机 ID (1 ~ 8)
 * @param raw_value: 协议原始值，如 8192 或 -10000
 */
void ESC_Control_Raw_Single(CAN_HandleTypeDef *hcan,uint8_t motor_id, int16_t raw_value) {
    uint8_t data[8] = {0};
    uint32_t can_id = 0;
    int data_index = 0;

    if (motor_id >= 1 && motor_id <= 4) {
        can_id = ESC_CAN_ID_GROUP_1;
        data_index = (motor_id - 1) * 2;
    } else if (motor_id >= 5 && motor_id <= 8) {
        can_id = ESC_CAN_ID_GROUP_2;
        data_index = (motor_id - 5) * 2;
    } else {
        return;
    }

    Int16_To_BigEndian(raw_value, &data[data_index], &data[data_index + 1]);
    CAN_Send_STD(hcan,can_id, data);
}

/**
 * @brief 原始值控制一组电机 (4个)
 * @param motor_start_id: 起始电机 ID (1 或 5)
 * @param raw_values: 长度为 4 的整型数组，对应 4 个电机的原始值
 */
void ESC_Control_Raw_Group(CAN_HandleTypeDef *hcan,uint8_t motor_start_id, int16_t raw_values[4]) {
    uint8_t data[8];
    uint32_t can_id = Get_CAN_ID(motor_start_id);
    
    if (can_id == 0) return;

    for (int i = 0; i < 4; i++) {
        Int16_To_BigEndian(raw_values[i], &data[i*2], &data[i*2 + 1]);
    }

    CAN_Send_STD(hcan,can_id, data);
}

/**
 * @brief 原始值控制所有电机 (8个)
 * @param raw_values: 长度为 8 的整型数组，索引 0 对应电机 1
 */
void ESC_Control_Raw_All(CAN_HandleTypeDef *hcan,int16_t raw_values[8]) {
    uint8_t data[8];

    // 发送第一帧 (电机 1-4)
    for (int i = 0; i < 4; i++) {
        Int16_To_BigEndian(raw_values[i], &data[i*2], &data[i*2 + 1]);
    }
    CAN_DJI_SendSTD(ESC_CAN_ID_GROUP_1, data);

    // 发送第二帧 (电机 5-8)
    for (int i = 0; i < 4; i++) {
        Int16_To_BigEndian(raw_values[i + 4], &data[i*2], &data[i*2 + 1]);
    }
    CAN_Send_STD(hcan,ESC_CAN_ID_GROUP_2, data);
}

