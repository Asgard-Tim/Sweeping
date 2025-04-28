#include "crash_sensor.h"
#include "stm32f4xx_hal.h"

void CrashSensor_Init(void)
{
    // CubeMX ������ PC14 (CRASH_L) �� PD12 (CRASH_R) Ϊ��������
    // ��������߼������ж����û�ȥ�����������ڴ�ʵ��
}

bool CrashSensor_Left(void)
{
    // ΢�����رպ�ʱ�� GPIO ���ͣ���ȡ GPIO_PIN_RESET ��ʾ��ײ
    return (HAL_GPIO_ReadPin(CRASH_L_GPIO_Port, CRASH_L_Pin) == GPIO_PIN_RESET);
}

bool CrashSensor_Right(void)
{
    return (HAL_GPIO_ReadPin(CRASH_R_GPIO_Port, CRASH_R_Pin) == GPIO_PIN_RESET);
}

uint8_t CrashSensor_GetStatus(void)
{
    uint8_t status = 0;
    if (CrashSensor_Left()) {
        status |= (1U << 0);
    }
    if (CrashSensor_Right()) {
        status |= (1U << 1);
    }
    return status;
}