// led.c
#include "led.h"

/* LED 初始化函数 */
void LED_Init(LED_HandleTypeDef *led) {
  // 根据LED的工作模式（主动低或主动高）初始化LED
  if (led->ActiveMode == LED_ACTIVE_LOW) {
    // 如果是主动低模式，初始化时将LED设置为关闭（对应的GPIO引脚输出高电平）
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_SET);
  } else {
    // 如果是主动高模式，初始化时将LED设置为关闭（对应的GPIO引脚输出低电平）
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_RESET);
  }
}

/* 点亮LED */
void LED_On(LED_HandleTypeDef *led) {
  if (led->ActiveMode == LED_ACTIVE_LOW) {
    // 如果是主动低模式，点亮LED需要将GPIO引脚输出低电平
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_RESET);
  } else {
    // 如果是主动高模式，点亮LED需要将GPIO引脚输出高电平
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_SET);
  }
}

/* 熄灭LED */
void LED_Off(LED_HandleTypeDef *led) {
  if (led->ActiveMode == LED_ACTIVE_LOW) {
    // 如果是主动低模式，熄灭LED需要将GPIO引脚输出高电平
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_SET);
  } else {
    // 如果是主动高模式，熄灭LED需要将GPIO引脚输出低电平
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_RESET);
  }
}

/* 切换LED状态（开/关） */
void LED_Toggle(LED_HandleTypeDef *led) {
  // 切换LED状态（如果是开就关，反之亦然）
  HAL_GPIO_TogglePin(led->GPIOx, led->GPIO_Pin);
}
