#ifndef __LED_H
#define __LED_H

#include "stm32f4xx_hal.h"  // 包含STM32 HAL库头文件

/* LED 工作模式枚举 */
typedef enum {
  LED_ACTIVE_LOW = 0,  // LED 在低电平时点亮（即连接到 VCC）
  LED_ACTIVE_HIGH      // LED 在高电平时点亮（即连接到 GND）
} LED_ActiveMode;

/* LED 句柄结构体定义 */
typedef struct {
  GPIO_TypeDef *GPIOx;      // GPIO 端口（例如 GPIOE）
  uint16_t GPIO_Pin;        // GPIO 引脚（例如 GPIO_PIN_14）
  LED_ActiveMode ActiveMode; // LED 工作模式（主动低或主动高）
} LED_HandleTypeDef;

/* LED 初始化函数 */
void LED_Init(LED_HandleTypeDef *led);

/* 点亮 LED 函数 */
void LED_On(LED_HandleTypeDef *led);

/* 熄灭 LED 函数 */
void LED_Off(LED_HandleTypeDef *led);

/* 切换 LED 状态（开/关）函数 */
void LED_Toggle(LED_HandleTypeDef *led);

#endif /* __LED_H */
