// led.c
#include "led.h"

/* LED ��ʼ������ */
void LED_Init(LED_HandleTypeDef *led) {
  // ����LED�Ĺ���ģʽ�������ͻ������ߣ���ʼ��LED
  if (led->ActiveMode == LED_ACTIVE_LOW) {
    // �����������ģʽ����ʼ��ʱ��LED����Ϊ�رգ���Ӧ��GPIO��������ߵ�ƽ��
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_SET);
  } else {
    // �����������ģʽ����ʼ��ʱ��LED����Ϊ�رգ���Ӧ��GPIO��������͵�ƽ��
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_RESET);
  }
}

/* ����LED */
void LED_On(LED_HandleTypeDef *led) {
  if (led->ActiveMode == LED_ACTIVE_LOW) {
    // �����������ģʽ������LED��Ҫ��GPIO��������͵�ƽ
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_RESET);
  } else {
    // �����������ģʽ������LED��Ҫ��GPIO��������ߵ�ƽ
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_SET);
  }
}

/* Ϩ��LED */
void LED_Off(LED_HandleTypeDef *led) {
  if (led->ActiveMode == LED_ACTIVE_LOW) {
    // �����������ģʽ��Ϩ��LED��Ҫ��GPIO��������ߵ�ƽ
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_SET);
  } else {
    // �����������ģʽ��Ϩ��LED��Ҫ��GPIO��������͵�ƽ
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_RESET);
  }
}

/* �л�LED״̬����/�أ� */
void LED_Toggle(LED_HandleTypeDef *led) {
  // �л�LED״̬������ǿ��͹أ���֮��Ȼ��
  HAL_GPIO_TogglePin(led->GPIOx, led->GPIO_Pin);
}
