#include "crash_sensor.h"
#include "stm32f4xx_hal.h"

void CrashSensor_Init(void)
{
    // CubeMX 已配置 PC14 (CRASH_L) 与 PD12 (CRASH_R) 为上拉输入
    // 若需额外逻辑（如中断配置或去抖处理），可在此实现
}

bool CrashSensor_Left(void)
{
    // 微动开关闭合时将 GPIO 拉低，读取 GPIO_PIN_RESET 表示碰撞
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