#ifndef HC05_H
#define HC05_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/**
 * @brief 初始化 HC-05（开始首次接收）
 * @param huart UART 句柄（例如 &huart3）
 */
void HC05_Init(UART_HandleTypeDef *huart);

/**
 * @brief 在 HAL_UART_RxCpltCallback 中调用此函数，处理收到的一个字节
 */
void HC05_UART_RxHandler(uint8_t byte);

/**
 * @brief 用户自定义的处理函数（弱定义，可重写）
 */
__weak void HC05_OnByteReceived(uint8_t byte);

uint8_t* HC05_GetRxBytePtr(void);

#endif
