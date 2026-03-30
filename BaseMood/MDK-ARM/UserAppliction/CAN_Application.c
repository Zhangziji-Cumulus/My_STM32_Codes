#include "CAN_Application.h"

MotorFeedback_t motor_feedback[8]; 

/**
  * @brief  解析电调反馈报文 (基于图片协议)
  * @param  std_id: CAN 标准帧 ID (例如 0x201)
  * @param  data: 8字节数据数组
  * @retval None
  */
void CAN_Parse_Motor_Feedback(uint32_t std_id, uint8_t* data)
{
    // 1. 检查 ID 是否在反馈范围内 (0x200 + ID)
    // 假设支持 ID 1 到 8，即 0x201 到 0x208
    if (std_id >= 0x201 && std_id <= 0x208) 
    {
        uint8_t index = std_id - 0x201; // 将 ID 映射到数组索引 0-7
        
        motor_feedback[index].id = index + 1;
        motor_feedback[index].is_online = true; // 收到数据，标记在线

        // 2. 解析角度 (DATA[0] 高8位, DATA[1] 低8位)
        // 范围 0-8191 对应 0-360度
        motor_feedback[index].angle_raw = (uint16_t)((data[0] << 8) | data[1]);
        
        // 转换为角度值 (可选，方便计算)
        motor_feedback[index].angle_deg = (float)motor_feedback[index].angle_raw * 360.0f / 8192.0f;

        // 3. 解析转速 (DATA[2] 高8位, DATA[3] 低8位)
        // 单位 rpm
        motor_feedback[index].speed_rpm = (int16_t)((data[2] << 8) | data[3]);

        // 4. 解析电流 (DATA[4] 高8位, DATA[5] 低8位)
        // 注意：图片未明确单位，但通常 DJI 电调反馈单位为 mA
        motor_feedback[index].current_ma = (int16_t)((data[4] << 8) | data[5]);

        // 5. 解析错误码 (DATA[7])
        motor_feedback[index].error_code = (MotorErrorCode_t)data[7];
        
        // DATA[6] 为空，忽略
    }
}
