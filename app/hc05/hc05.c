#include "hc05.h"

UART_HandleTypeDef *hc05_uart = NULL;
uint8_t hc05_rx_byte = 0;

void HC05_Init(UART_HandleTypeDef *huart)
{
    hc05_uart = huart;
    HAL_UART_Receive_IT(hc05_uart, &hc05_rx_byte, 1);
}

void HC05_UART_RxHandler(uint8_t byte)
{
    HC05_OnByteReceived(byte); // 调用用户处理函数

    // 继续接收
    if (hc05_uart != NULL)
        HAL_UART_Receive_IT(hc05_uart, &hc05_rx_byte, 1);
}

// 可由用户在 main.c 重写
__weak void HC05_OnByteReceived(uint8_t byte)
{
    // 默认空实现
}

uint8_t* HC05_GetRxBytePtr(void)
{
    return &hc05_rx_byte;
}
