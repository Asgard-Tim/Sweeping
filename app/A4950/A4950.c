// A4950.c
#include "stm32f4xx_hal.h"
#include "A4950.h"

void A4950_Init(void)
{
    // 启动左电机 PWM（TIM1_CH1=PE9, TIM1_CH2=PE11）
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    // 启动右电机 PWM（TIM9_CH1=PE5, TIM9_CH2=PE6）
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
}

void A4950_SetLeft(int16_t speed)
{
    uint16_t duty;
    if (speed >= 0) {
        duty = (speed > A4950_PWM_MAX ? A4950_PWM_MAX : speed);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    } else {
        duty = ((-speed) > A4950_PWM_MAX ? A4950_PWM_MAX : -speed);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
    }
}

void A4950_SetRight(int16_t speed)
{
    uint16_t duty;
    if (speed >= 0) {
        duty = (speed > A4950_PWM_MAX ? A4950_PWM_MAX : speed);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, duty);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
    } else {
        duty = ((-speed) > A4950_PWM_MAX ? A4950_PWM_MAX : -speed);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, duty);
    }
}

void A4950_Brake(void)
{
    // 两输入同时高 → 主动制动
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, A4950_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, A4950_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, A4950_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, A4950_PWM_MAX);
}
