#ifndef HC05_H
#define HC05_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/**
 * @brief ��ʼ�� HC-05����ʼ�״ν��գ�
 * @param huart UART ��������� &huart3��
 */
void HC05_Init(UART_HandleTypeDef *huart);

/**
 * @brief �� HAL_UART_RxCpltCallback �е��ô˺����������յ���һ���ֽ�
 */
void HC05_UART_RxHandler(uint8_t byte);

/**
 * @brief �û��Զ���Ĵ������������壬����д��
 */
__weak void HC05_OnByteReceived(uint8_t byte);

uint8_t* HC05_GetRxBytePtr(void);

#endif
