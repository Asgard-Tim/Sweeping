// A4950.h
#ifndef __A4950_H
#define __A4950_H

#include "stm32f4xx_hal.h"

/* ������ TIM_HandleTypeDef �� main.c �ж��壨�� extern����
 * ������Ҫ������ extern���Ա��� A4950.c �����ã�
 */
extern TIM_HandleTypeDef htim1;  // ����������PE9(Ch1)/PE11(Ch2)
extern TIM_HandleTypeDef htim9;  // �����ҵ����PE5(Ch1)/PE6(Ch2)

/** PWM ���ռ�ձȣ���Ӧ TIMx->ARR ֵ */
#define A4950_PWM_MAX  (htim1.Init.Period)

/**
 * @brief  ���� A4950 �� PWM �����
 *         ����ǰ���� main.c ��ִ�� MX_TIM1_Init()��MX_TIM9_Init()��
 */
void A4950_Init(void);

/**
 * @brief  ���������ٶȣ�PE9/PE11����
 * @param  speed: -A4950_PWM_MAX �� +A4950_PWM_MAX��
 *                ��ֵǰ������ֵ���ˣ����޻ᱻ�ضϡ�
 */
void A4950_SetLeft(int16_t speed);

/**
 * @brief  �����ҵ���ٶȣ�PE5/PE6����
 * @param  speed: -A4950_PWM_MAX �� +A4950_PWM_MAX��
 *                ��ֵǰ������ֵ���ˣ����޻ᱻ�ضϡ�
 */
void A4950_SetRight(int16_t speed);

/**
 * @brief  �����ƶ���Brake ģʽ����AIN1/AIN2��BIN1/BIN2 ͬʱ�ߡ�
 */
void A4950_Brake(void);

#endif // __A4950_H
