#ifndef __LED_H
#define __LED_H

#include "stm32f4xx_hal.h"  // ����STM32 HAL��ͷ�ļ�

/* LED ����ģʽö�� */
typedef enum {
  LED_ACTIVE_LOW = 0,  // LED �ڵ͵�ƽʱ�����������ӵ� VCC��
  LED_ACTIVE_HIGH      // LED �ڸߵ�ƽʱ�����������ӵ� GND��
} LED_ActiveMode;

/* LED ����ṹ�嶨�� */
typedef struct {
  GPIO_TypeDef *GPIOx;      // GPIO �˿ڣ����� GPIOE��
  uint16_t GPIO_Pin;        // GPIO ���ţ����� GPIO_PIN_14��
  LED_ActiveMode ActiveMode; // LED ����ģʽ�������ͻ������ߣ�
} LED_HandleTypeDef;

/* LED ��ʼ������ */
void LED_Init(LED_HandleTypeDef *led);

/* ���� LED ���� */
void LED_On(LED_HandleTypeDef *led);

/* Ϩ�� LED ���� */
void LED_Off(LED_HandleTypeDef *led);

/* �л� LED ״̬����/�أ����� */
void LED_Toggle(LED_HandleTypeDef *led);

#endif /* __LED_H */
