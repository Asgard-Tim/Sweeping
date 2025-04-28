// A4950.h
#ifndef __A4950_H
#define __A4950_H

#include "stm32f4xx_hal.h"

/* 如果你的 TIM_HandleTypeDef 在 main.c 中定义（非 extern），
 * 这里需要声明成 extern，以便在 A4950.c 中引用：
 */
extern TIM_HandleTypeDef htim1;  // 用于左电机：PE9(Ch1)/PE11(Ch2)
extern TIM_HandleTypeDef htim9;  // 用于右电机：PE5(Ch1)/PE6(Ch2)

/** PWM 最大占空比，对应 TIMx->ARR 值 */
#define A4950_PWM_MAX  (htim1.Init.Period)

/**
 * @brief  启动 A4950 的 PWM 输出。
 *         调用前请在 main.c 中执行 MX_TIM1_Init()、MX_TIM9_Init()。
 */
void A4950_Init(void);

/**
 * @brief  设置左电机速度（PE9/PE11）。
 * @param  speed: -A4950_PWM_MAX … +A4950_PWM_MAX，
 *                正值前进，负值后退，超限会被截断。
 */
void A4950_SetLeft(int16_t speed);

/**
 * @brief  设置右电机速度（PE5/PE6）。
 * @param  speed: -A4950_PWM_MAX … +A4950_PWM_MAX，
 *                正值前进，负值后退，超限会被截断。
 */
void A4950_SetRight(int16_t speed);

/**
 * @brief  主动制动（Brake 模式），AIN1/AIN2、BIN1/BIN2 同时高。
 */
void A4950_Brake(void);

#endif // __A4950_H
