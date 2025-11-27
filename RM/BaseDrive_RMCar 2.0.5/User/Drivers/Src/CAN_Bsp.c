#include "CAN_Bsp.h"
#include <string.h>

//** 外部变量 **//
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//** 本地宏定义 **//

//** 本地变量声明 **//

/* CAN1过滤ID */
static uint32_t Can1_std_ids[] = {
	0x201,
	0x202, 
	0x203, 
	0x204,
	0x205,
	0x206,
	0x207,
	0x208
};
/* 配置CAN1过滤器结构体 */
static CAN_FilterConfig can1_config = {
	 .hcan = &hcan1,
	 .frame_type = CAN_FRAME_STD,
	 .accept_ids = Can1_std_ids,
	 .id_count = (sizeof(Can1_std_ids) / sizeof(uint32_t))
};

//** 本地函数声明 **//

/* CAN配置过滤器通用函数 */
static HAL_StatusTypeDef CAN_Filter_Config_UseBestScale(CAN_FilterConfig *config);
/* CAN过滤器接收所有ID */
static HAL_StatusTypeDef CAN_Config_AcceptAllID(CAN_HandleTypeDef *hcan, uint8_t filter_bank);


//** 对外函数 **//

void My_CAN_Init(void)
{
	CAN_Filter_Config_UseBestScale(&can1_config);
	CAN_Config_AcceptAllID(&hcan2,14);
}

/* CAN发送函数 */
/**
* @brief  CAN发送函数（使用结构体参数封装）
* @param  msg_config: CAN发送参数结构体指针
* @retval HAL_StatusTypeDef: 发送结果 (HAL_OK/HAL_ERROR等)
*/
HAL_StatusTypeDef CAN_SendMessage(CAN_Send_Msg_Config_t *msg_config)
{
    // 参数合法性检查（增加结构体指针空指针检查）
    if (msg_config == NULL) {
        return HAL_ERROR;
    }
    if (msg_config->hcan == NULL || msg_config->data == NULL) {
        return HAL_ERROR;  // 句柄或数据指针无效
    }
    if (msg_config->data_len > 8) {
        return HAL_ERROR;  // CAN数据长度最大为8字节
    }
    if (!msg_config->is_ext_id && (msg_config->std_id > 0x7FF)) {
        return HAL_ERROR;  // 标准ID超出范围
    }
    if (msg_config->is_ext_id && (msg_config->ext_id > 0x1FFFFFFF)) {
        return HAL_ERROR;  // 扩展ID超出范围
    }

    CAN_TxHeaderTypeDef TxHeader;  // 发送消息头
    uint32_t TxMailbox;            // 发送邮箱编号

    // 配置消息头
    TxHeader.StdId = msg_config->std_id;                   // 标准ID
    TxHeader.ExtId = msg_config->ext_id;                   // 扩展ID
    TxHeader.RTR = CAN_RTR_DATA;                           // 数据帧（非远程帧）
    TxHeader.IDE = msg_config->is_ext_id ? CAN_ID_EXT : CAN_ID_STD;  // ID类型
    TxHeader.DLC = msg_config->data_len;                   // 数据长度
    TxHeader.TransmitGlobalTime = DISABLE;                 // 不启用发送时间戳

    // 调用HAL库发送函数
    return HAL_CAN_AddTxMessage(msg_config->hcan, &TxHeader, msg_config->data, &TxMailbox);
}

//** 本地函数 **//

/* CAN配置过滤器通用函数 */
/**
* 功能：配置CAN过滤器，传入CAN_FilterConfig结构体，支持标准帧和扩展帧
*/
static HAL_StatusTypeDef CAN_Filter_Config_UseBestScale(CAN_FilterConfig *config)
{
    if (!config || !config->hcan || !config->accept_ids || config->id_count == 0) {
        return HAL_ERROR;
    }

    CAN_HandleTypeDef *hcan = config->hcan;
    CAN_FilterTypeDef filter;
    memset(&filter, 0, sizeof(filter));

    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14; // F1/F4 需要设置；H7 不需要

    uint8_t bank_idx = 0;

    if (config->frame_type == CAN_FRAME_STD) {
        // 使用 16bit 模式：每 bank 存两个 11bit 标准 ID
        filter.FilterMode = CAN_FILTERMODE_IDLIST;
        filter.FilterScale = CAN_FILTERSCALE_16BIT;

        for (uint8_t i = 0; i < config->id_count; i += 2) {
            if (bank_idx >= 14) return HAL_ERROR; // 超出硬件限制

            filter.FilterBank = bank_idx++;

            // 第一个 ID
            filter.FilterIdHigh = (uint16_t)((config->accept_ids[i] << 5) & 0xFFE0); // 左移5位对齐

            // 第二个 ID（如果存在）
            if (i + 1 < config->id_count) {
                filter.FilterIdLow = (uint16_t)((config->accept_ids[i + 1] << 5) & 0xFFE0);
            } else {
                filter.FilterIdLow = 0x0000; // 填充
            }

            // Mask 字段在列表模式中可设为 0
            filter.FilterMaskIdHigh = 0x0000;
            filter.FilterMaskIdLow  = 0x0000;

            if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK) {
                return HAL_ERROR;
            }
        }
    }
    else if (config->frame_type == CAN_FRAME_EXT) {
        // 使用 32bit 模式：每 bank 存一个 29bit 扩展 ID
        filter.FilterMode = CAN_FILTERMODE_IDLIST;
        filter.FilterScale = CAN_FILTERSCALE_32BIT;

        for (uint8_t i = 0; i < config->id_count; i++) {
            if (bank_idx >= 14) return HAL_ERROR; // 最多 14 个 bank

            filter.FilterBank = bank_idx++;

            uint32_t can_id = config->accept_ids[i];

            // 高16位: ID[28:13]
            filter.FilterIdHigh = (uint16_t)(can_id >> 13);
            // 低16位: ID[12:0] 左移3位，并设置 IDE=1
            filter.FilterIdLow  = (uint16_t)((can_id << 3) & 0xFFF8) | CAN_ID_EXT;

            // 列表模式下 mask 不参与匹配，但必须写入（可全0）
            filter.FilterMaskIdHigh = 0x0000;
            filter.FilterMaskIdLow  = 0x0000;

            if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK) {
                return HAL_ERROR;
            }
        }
    }
    else {
        return HAL_ERROR; // 无效类型
    }

    // 启动 CAN
    if (HAL_CAN_Start(hcan) != HAL_OK) {
        return HAL_ERROR;
    }

    // 启用 FIFO0 中断
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* CAN过滤器接收所有ID */
/**
 * @brief  CAN接收所有ID配置函数（标准帧+扩展帧全接收，硬件过滤模式）
 * @param  hcan: CAN外设句柄（&hcan1 / &hcan2）
 * @param  filter_bank: 指定使用的过滤器组（CAN1:0~13，CAN2:14~27，推荐CAN1=0，CAN2=14）
 * @retval HAL_StatusTypeDef: HAL_OK=成功，HAL_ERROR=参数错误/配置失败
 * @note   1. 仅占用1个过滤器组，即可接收所有ID
 *         2. 配置后需确保CAN外设已完成底层初始化（时钟/GPIO/波特率）
 */
static HAL_StatusTypeDef CAN_Config_AcceptAllID(CAN_HandleTypeDef *hcan, uint8_t filter_bank)
{
	// 1. 核心参数校验
    if (hcan == NULL) return HAL_ERROR;

    // 2. 过滤器组合法性校验（严格遵守CAN1/CAN2分配规则）
    if (hcan->Instance == CAN1 && (filter_bank > 13 || filter_bank < 0)) {
        return HAL_ERROR; // CAN1只能用0~13组
    }
    if (hcan->Instance == CAN2 && (filter_bank < 14 || filter_bank > 27)) {
        return HAL_ERROR; // CAN2只能用14~27组
    }
	// 3. 配置CAN过滤器（掩码模式+全0掩码=接收所有ID）
	CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	
    can_filter_st.SlaveStartFilterBank = filter_bank;
    can_filter_st.FilterBank = filter_bank;

    // 4. 应用过滤器配置
    if (HAL_CAN_ConfigFilter(hcan, &can_filter_st) != HAL_OK) {
        return HAL_ERROR;
    }

    // 5. 启动CAN外设（确保CAN处于工作状态）
    if (HAL_CAN_Start(hcan) != HAL_OK) {
            return HAL_ERROR;
    }
    
    // 6. 开启FIFO0接收中断（可选，推荐开启，提升实时性）
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}


///**
// * @brief  CAN发送函数（支持指定CAN句柄和消息参数）
// * @param  hcan: CAN外设句柄指针（如&hcan1, &hcan2）
// * @param  std_id: 标准ID (11位，0~0x7FF)
// * @param  ext_id: 扩展ID (29位，0~0x1FFFFFFF)
// * @param  is_ext_id: 是否使用扩展ID (0=标准ID, 1=扩展ID)
// * @param  data_len: 数据长度 (0~8)
// * @param  data: 待发送的数据指针
// * @retval HAL_StatusTypeDef: 发送结果 (HAL_OK/HAL_ERROR等)
// */
//HAL_StatusTypeDef CAN_SendMessage(CAN_HandleTypeDef *hcan, 
//                                 uint32_t std_id, 
//                                 uint32_t ext_id, 
//                                 uint8_t is_ext_id, 
//                                 uint8_t data_len, 
//                                 uint8_t *data)
//{
//    // 参数合法性检查
//    if (hcan == NULL || data == NULL) {
//        return HAL_ERROR;  // 句柄或数据指针无效
//    }
//    if (data_len > 8) {
//        return HAL_ERROR;  // CAN数据长度最大为8字节
//    }
//    if (!is_ext_id && (std_id > 0x7FF)) {
//        return HAL_ERROR;  // 标准ID超出范围
//    }
//    if (is_ext_id && (ext_id > 0x1FFFFFFF)) {
//        return HAL_ERROR;  // 扩展ID超出范围
//    }

//    CAN_TxHeaderTypeDef TxHeader;  // 发送消息头
//    uint32_t TxMailbox;            // 发送邮箱编号

//    // 配置消息头
//    TxHeader.StdId = std_id;                   // 标准ID
//    TxHeader.ExtId = ext_id;                   // 扩展ID
//    TxHeader.RTR = CAN_RTR_DATA;               // 数据帧（非远程帧）
//    TxHeader.IDE = is_ext_id ? CAN_ID_EXT : CAN_ID_STD;  // ID类型（标准/扩展）
//    TxHeader.DLC = data_len;                   // 数据长度
//    TxHeader.TransmitGlobalTime = DISABLE;     // 不启用发送时间戳

//    // 调用HAL库发送函数，使用指定的CAN句柄
//    return HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox);
//}

//** 本地函数 **//



