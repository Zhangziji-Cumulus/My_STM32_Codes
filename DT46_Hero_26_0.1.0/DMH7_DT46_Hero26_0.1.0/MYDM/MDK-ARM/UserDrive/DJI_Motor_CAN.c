#include "DJI_Motor_CAN.h" // 【修改】包含新的头文件

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
DJI_MotorFeedback_t DJI_MFeedback_CAN1[8];
DJI_MotorFeedback_t DJI_MFeedback_CAN2[8];
DJI_MotorFeedback_t DJI_MFeedback_CAN3[8];

//* 对外函数 *//
/**
  * @brief  解析电调反馈报文 (基于图片协议)
  * @param  DJI_MFeedback: 电机反馈结构体数组指针
  * @param  std_id: CAN 标准帧 ID (例如 0x201)
  * @param  data: 8字节数据数组
  * @retval None
  */
void CAN_DJI_Motor_Feedback(DJI_MotorFeedback_t* DJI_MFeedback, uint32_t std_id, uint8_t* data)
{
    // 【注意】此函数完全不需要修改，因为只做数据解析，和硬件无关
    if (std_id >= 0x201 && std_id <= 0x208) 
    {
        uint8_t index = std_id - 0x201;
        
        DJI_MFeedback[index].id = index + 1;
        DJI_MFeedback[index].is_online = true;

        // 2. 解析角度
        DJI_MFeedback[index].angle_raw = (uint16_t)((data[0] << 8) | data[1]);
        DJI_MFeedback[index].angle_deg = (float)DJI_MFeedback[index].angle_raw * 360.0f / 8192.0f;

        // 3. 解析转速
        DJI_MFeedback[index].speed_rpm = (int16_t)((data[2] << 8) | data[3]);

        // 4. 解析电流
        DJI_MFeedback[index].current_ma = (int16_t)((data[4] << 8) | data[5]);

        // 5. 解析错误码
        DJI_MFeedback[index].error_code = (DJI_MotorErrorCode_t)data[7];
    }
}

//** 控制电机函数 **//

//* 对外函数 *//

/* ================= 电调配置实例 ================= */

const ESC_Config_t ESC_C620_20A = {
    .name = "ESC_20A_Protocol",
    .max_current_amps = 20.0f,
    .max_raw_value = 16384
};

const ESC_Config_t ESC_C610_10A = {
    .name = "ESC_10A_Protocol",
    .max_current_amps = 10.0f,
    .max_raw_value = 10000
};

/* ================= 内部辅助函数 ================= */

static void Int16_To_BigEndian(int16_t value, uint8_t* high, uint8_t* low) {
    uint16_t u_value = (uint16_t)value;
    *high = (u_value >> 8) & 0xFF;
    *low  = u_value & 0xFF;
}

static int16_t Amps_To_Raw(const ESC_Config_t* config, float amps) {
    if (config == NULL || config->max_current_amps <= 0) {
        return 0;
    }

    float ratio = (float)config->max_raw_value / config->max_current_amps;
    int32_t raw_val_32 = (int32_t)(amps * ratio);
    
    if (raw_val_32 > config->max_raw_value) 
        raw_val_32 = config->max_raw_value;
    if (raw_val_32 < -config->max_raw_value) 
        raw_val_32 = -config->max_raw_value;
    if (raw_val_32 > 32767) raw_val_32 = 32767;
    if (raw_val_32 < -32768) raw_val_32 = -32768;

    return (int16_t)raw_val_32;
}

static uint32_t Get_CAN_ID(uint8_t motor_start_id) {
    if (motor_start_id == 1) return ESC_CAN_ID_GROUP_1;
    if (motor_start_id == 5) return ESC_CAN_ID_GROUP_2;
    return 0;
}

/* ================= 底层发送 (FDCAN HAL) ================= */

/**
 * @brief FDCAN 底层发送标准帧函数
 * @param hfdcan: FDCAN 句柄
 * @param id: CAN ID
 * @param data: 8字节数据
 */
void FDCAN_DJI_SendSTD(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t* data) {
    // 【修改】直接调用我们之前移植好的 FDCAN_Send_STD
    FDCAN_Send_STD(hfdcan, id, data);
}

/* ================= 安培值控制函数 ================= */

/**
 * @brief 安培值控制单个电机
 * @param hfdcan: FDCAN 句柄指针
 * @param config: 电调配置指针
 * @param motor_id: 电机 ID (1 ~ 8)
 * @param current_amps: 目标电流 (A)
 */
void ESC_Control_Amps_Single(FDCAN_HandleTypeDef *hfdcan, const ESC_Config_t* config, uint8_t motor_id, float current_amps){
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

    int16_t raw_val = Amps_To_Raw(config, current_amps);
    Int16_To_BigEndian(raw_val, &data[data_index], &data[data_index + 1]);
    
    // 【修改】调用 FDCAN 发送函数
    FDCAN_Send_STD(hfdcan, can_id, data);
}

/**
 * @brief 安培值控制一组电机 (4个)
 */
void ESC_Control_Amps_Group(FDCAN_HandleTypeDef *hfdcan, const ESC_Config_t* config, uint8_t motor_start_id, float currents_amps[4]) {
    uint8_t data[8] = {0};
    uint32_t can_id = Get_CAN_ID(motor_start_id);
    
    if (can_id == 0) return;

    for (int i = 0; i < 4; i++) {
        int16_t raw_val = Amps_To_Raw(config, currents_amps[i]);
        Int16_To_BigEndian(raw_val, &data[i*2], &data[i*2 + 1]);
    }

    // 【修改】调用 FDCAN 发送函数
    FDCAN_Send_STD(hfdcan, can_id, data);
}

/**
 * @brief 安培值控制所有电机 (8个)
 * @note 我帮你把之前注释掉的代码也恢复并移植好了
 */
void ESC_Control_Amps_All(FDCAN_HandleTypeDef *hfdcan, const ESC_Config_t* config, float currents_amps[8]) {
    uint8_t data[8] = {0};

    // 发送第一帧 (电机 1-4)
    for (int i = 0; i < 4; i++) {
        int16_t raw_val = Amps_To_Raw(config, currents_amps[i]);
        Int16_To_BigEndian(raw_val, &data[i*2], &data[i*2 + 1]);
    }
    FDCAN_Send_STD(hfdcan, ESC_CAN_ID_GROUP_1, data);

    // 发送第二帧 (电机 5-8)
    for (int i = 0; i < 4; i++) {
        int16_t raw_val = Amps_To_Raw(config, currents_amps[i + 4]);
        Int16_To_BigEndian(raw_val, &data[i*2], &data[i*2 + 1]);
    }
    FDCAN_Send_STD(hfdcan, ESC_CAN_ID_GROUP_2, data);
}

/* ================= 原始值控制函数 ================= */

/**
 * @brief 原始值控制单个电机
 */
void ESC_Control_Raw_Single(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int16_t raw_value) {
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
    
    // 【修改】调用 FDCAN 发送函数
    FDCAN_Send_STD(hfdcan, can_id, data);
}

/**
 * @brief 原始值控制一组电机 (4个)
 */
void ESC_Control_Raw_Group(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_start_id, int16_t raw_values[4]) {
    uint8_t data[8];
    uint32_t can_id = Get_CAN_ID(motor_start_id);
    
    if (can_id == 0) return;

    for (int i = 0; i < 4; i++) {
        Int16_To_BigEndian(raw_values[i], &data[i*2], &data[i*2 + 1]);
    }

    // 【修改】调用 FDCAN 发送函数
    FDCAN_Send_STD(hfdcan, can_id, data);
}

/**
 * @brief 原始值控制所有电机 (8个)
 */
void ESC_Control_Raw_All(FDCAN_HandleTypeDef *hfdcan, int16_t raw_values[8]) {
    uint8_t data[8];

    // 发送第一帧 (电机 1-4)
    for (int i = 0; i < 4; i++) {
        Int16_To_BigEndian(raw_values[i], &data[i*2], &data[i*2 + 1]);
    }
    FDCAN_Send_STD(hfdcan, ESC_CAN_ID_GROUP_1, data);

    // 发送第二帧 (电机 5-8)
    for (int i = 0; i < 4; i++) {
        Int16_To_BigEndian(raw_values[i + 4], &data[i*2], &data[i*2 + 1]);
    }
    FDCAN_Send_STD(hfdcan, ESC_CAN_ID_GROUP_2, data);
}