// led.c
#include "led.h"

/* ???LED */
void LED_Init(LED_HandleTypeDef *led) {
  // ????????????(????)
  if (led->ActiveMode == LED_ACTIVE_LOW) {
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_SET); // ?????
  } else {
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_RESET); // ?????
  }
}

/* ??LED */
void LED_On(LED_HandleTypeDef *led) {
  if (led->ActiveMode == LED_ACTIVE_LOW) {
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_RESET); // ?????
  } else {
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_SET);   // ?????
  }
}

/* ??LED */
void LED_Off(LED_HandleTypeDef *led) {
  if (led->ActiveMode == LED_ACTIVE_LOW) {
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_SET);   // ?????
  } else {
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_RESET); // ?????
  }
}

/* ??LED?? */
void LED_Toggle(LED_HandleTypeDef *led) {
  HAL_GPIO_TogglePin(led->GPIOx, led->GPIO_Pin);
}

