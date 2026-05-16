#include "bsp_SBUS.h"

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *sbusHuart;
static uint8_t sbusRxBuffer[SBUS_RX_BUF_SIZE]; /* DMA 循环接收缓冲区 */

/* Private data --------------------------------------------------------------*/
static SbusData_t sbusData;

/* Private function prototypes ----------------------------------------------*/
static void SBUS_DecodeFrame(const uint8_t *frameData);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 SBUS 接收
 * @param huart SBUS 对应的 UART 句柄
 */
void SBUS_Init(UART_HandleTypeDef *huart) {
    sbusHuart = huart;

    memset(&sbusData, 0, sizeof(sbusData));
    memset(sbusRxBuffer, 0, sizeof(sbusRxBuffer));

    /* 启用 UART IDLE 中断 */
    __HAL_UART_ENABLE_IT(sbusHuart, UART_IT_IDLE);

    /* 启用 DMA 循环接收 */
    HAL_UART_Receive_DMA(sbusHuart, sbusRxBuffer, SBUS_RX_BUF_SIZE);
}

/**
 * @brief 获取当前遥控器数据的只读指针
 * @return const SbusData_t*
 */
const SbusData_t *get_SBUS_Data_point(void)
{
    return &sbusData;
}

/**
 * @brief 处理 SBUS 数据帧，需在主循环中定期调用
 */
void SBUS_Process(void) {
    uint16_t i;

    if (sbusData.newDataFlag || (HAL_GetTick() - sbusData.lastParseTick > 5)) {
        for (i = 0; i <= SBUS_RX_BUF_SIZE - SBUS_FRAME_LEN; i++) {
            if (sbusRxBuffer[i] == SBUS_HEADER && sbusRxBuffer[i + 24] == SBUS_FOOTER) {
                SBUS_DecodeFrame(&sbusRxBuffer[i]);
                break;
            }
        }

        sbusData.newDataFlag = 0;
        sbusData.lastParseTick = HAL_GetTick();
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 解码一帧 SBUS 数据
 * @param d 指向帧头的指针
 */
static void SBUS_DecodeFrame(const uint8_t *d) {
    sbusData.channels[0]  = (((uint16_t)d[1]  << 0)  | ((uint16_t)d[2]  << 8)) & 0x07FF;
    sbusData.channels[1]  = (((uint16_t)d[2]  >> 3)  | ((uint16_t)d[3]  << 5)) & 0x07FF;
    sbusData.channels[2]  = (((uint16_t)d[3]  >> 6)  | ((uint16_t)d[4]  << 2)  | ((uint16_t)d[5] << 10)) & 0x07FF;
    sbusData.channels[3]  = (((uint16_t)d[5]  >> 1)  | ((uint16_t)d[6]  << 7)) & 0x07FF;
    sbusData.channels[4]  = (((uint16_t)d[6]  >> 4)  | ((uint16_t)d[7]  << 4)) & 0x07FF;
    sbusData.channels[5]  = (((uint16_t)d[7]  >> 7)  | ((uint16_t)d[8]  << 1)  | ((uint16_t)d[9] << 9))  & 0x07FF;
    sbusData.channels[6]  = (((uint16_t)d[9]  >> 2)  | ((uint16_t)d[10] << 6)) & 0x07FF;
    sbusData.channels[7]  = (((uint16_t)d[10] >> 5)  | ((uint16_t)d[11] << 3)) & 0x07FF;

    sbusData.channels[8]  = (((uint16_t)d[12] << 0)  | ((uint16_t)d[13] << 8)) & 0x07FF;
    sbusData.channels[9]  = (((uint16_t)d[13] >> 3)  | ((uint16_t)d[14] << 5)) & 0x07FF;
    sbusData.channels[10] = (((uint16_t)d[14] >> 6)  | ((uint16_t)d[15] << 2)  | ((uint16_t)d[16] << 10)) & 0x07FF;
    sbusData.channels[11] = (((uint16_t)d[16] >> 1)  | ((uint16_t)d[17] << 7)) & 0x07FF;
    sbusData.channels[12] = (((uint16_t)d[17] >> 4)  | ((uint16_t)d[18] << 4)) & 0x07FF;
    sbusData.channels[13] = (((uint16_t)d[18] >> 7)  | ((uint16_t)d[19] << 1)  | ((uint16_t)d[20] << 9))  & 0x07FF;
    sbusData.channels[14] = (((uint16_t)d[20] >> 2)  | ((uint16_t)d[21] << 6)) & 0x07FF;
    sbusData.channels[15] = (((uint16_t)d[21] >> 5)  | ((uint16_t)d[22] << 3)) & 0x07FF;

    sbusData.ch17      = (d[23] >> 0) & 0x01;
    sbusData.ch18      = (d[23] >> 1) & 0x01;
    sbusData.frameLost = (d[23] >> 2) & 0x01;
    sbusData.failsafe  = (d[23] >> 3) & 0x01;
}

/**
 * @brief 检查遥控器是否掉线
 * @return 1 表示掉线，0 表示正常
 */
uint8_t SBUS_IsRemoteLost(void)
{
    return (sbusData.failsafe || sbusData.frameLost);
}

/**
 * @brief SBUS UART IDLE 中断处理函数
 * @note 需在 USARTx_IRQHandler 中调用
 */
void SBUS_UART_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(sbusHuart, UART_FLAG_IDLE) != RESET) {
        (void)sbusHuart->Instance->SR;
        (void)sbusHuart->Instance->DR;
        sbusData.newDataFlag = 1;
    }
}
