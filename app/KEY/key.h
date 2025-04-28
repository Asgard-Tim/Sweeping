// key.h
#ifndef __KEY_H
#define __KEY_H

#include "stm32f4xx_hal.h"

/* 按键工作电平模式 */
typedef enum {
    KEY_ACTIVE_LOW = 0,  // 低电平有效（按下接GND）
    KEY_ACTIVE_HIGH      // 高电平有效（按下接VCC）
} Key_ActiveMode;

/* 按键状态 */
typedef enum {
    KEY_STATE_RELEASED = 0,      // 松开状态
    KEY_STATE_PRESSED,           // 按下状态
    KEY_STATE_JUST_PRESSED,      // 刚刚按下（边沿）
    KEY_STATE_JUST_RELEASED      // 刚刚松开（边沿）
} Key_State;

/* 按键句柄结构体 */
typedef struct {
    GPIO_TypeDef *GPIOx;          // GPIO端口（如GPIOD）
    uint16_t GPIO_Pin;            // GPIO引脚（如GPIO_PIN_11）
    Key_ActiveMode ActiveMode;    // 有效电平模式
    Key_State CurrentState;       // 当前状态
    uint32_t LastTick;            // 最后一次状态变化时的时间戳
} Key_HandleTypeDef;

/* 函数声明 */
void Key_Init(Key_HandleTypeDef *key);
void Key_Update(Key_HandleTypeDef *key);
Key_State Key_GetState(Key_HandleTypeDef *key);

#endif /* __KEY_H */
