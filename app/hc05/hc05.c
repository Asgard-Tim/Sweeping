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
    HC05_OnByteReceived(byte); // �����û�������

    // ��������
    if (hc05_uart != NULL)
        HAL_UART_Receive_IT(hc05_uart, &hc05_rx_byte, 1);
}

// �����û��� main.c ��д
__weak void HC05_OnByteReceived(uint8_t byte)
{
    // Ĭ�Ͽ�ʵ��
}

uint8_t* HC05_GetRxBytePtr(void)
{
    return &hc05_rx_byte;
}
