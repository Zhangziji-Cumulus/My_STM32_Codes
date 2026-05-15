#ifndef __BSP_SBUS_H__
#define __BSP_SBUS_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported constants --------------------------------------------------------*/
#define SBUS_FRAME_LEN         25      /**< SBUS 标准帧长度 */
#define SBUS_RX_BUF_SIZE       50      /**< DMA 接收缓冲区大小，建议为帧长的2倍 */
#define SBUS_HEADER            0x0F    /**< SBUS 帧头 */
#define SBUS_FOOTER            0x00    /**< SBUS 帧尾 */

/* Exported types ------------------------------------------------------------*/ 
typedef struct {
    uint16_t channels[16];        /**< 16个标准通道，值范围 172-1811，中立值约 1024 */
    uint8_t ch17;                 /**< 数字通道 17 */
    uint8_t ch18;                 /**< 数字通道 18 */
    uint8_t frameLost;            /**< 丢帧标志 */
    uint8_t failsafe;             /**< 失控保护标志 */
    uint8_t newDataFlag;          /**< 中断级新数据到达标志 */
    uint32_t lastParseTick;       /**< 上次解析时间戳，用于超时检测 */
} SbusData_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 SBUS 接收
 *
 * @param huart UART 句柄
 */
void SBUS_Init(UART_HandleTypeDef *huart);

/**
 * @brief 获取遥控器数据指针（只读）
 *
 * @return const SbusData_t* 指向当前遥控器数据的指针
 */
const SbusData_t *get_remote_control_point(void);

/**
 * @brief 处理 SBUS 数据，需在主循环中高频调用
 */
void SBUS_Process(void);

/**
 * @brief 检查遥控器是否掉线
 *
 * @return 1 表示掉线，0 表示正常
 */
uint8_t SBUS_IsRemoteLost(void);

#endif /* __BSP_SBUS_H__ */
