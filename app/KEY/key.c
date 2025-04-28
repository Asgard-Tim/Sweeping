// key.c
#include "key.h"
#include "main.h"  // 需要 HAL_GetTick()

#define DEBOUNCE_DELAY_MS 50  // 消抖延迟（单位：毫秒）

/* 按键初始化 */
void Key_Init(Key_HandleTypeDef *key) {
    key->CurrentState = KEY_STATE_RELEASED;
    key->LastTick = HAL_GetTick();
}

/* 按键更新（需要定期调用，比如10ms一次） */
void Key_Update(Key_HandleTypeDef *key) {
    uint32_t current_tick = HAL_GetTick();
    
    // 读取当前按键引脚状态
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(key->GPIOx, key->GPIO_Pin);
    uint8_t is_pressed = (key->ActiveMode == KEY_ACTIVE_LOW) ? 
                         (pin_state == GPIO_PIN_RESET) : 
                         (pin_state == GPIO_PIN_SET);

    // 按键状态机
    switch (key->CurrentState) {
        case KEY_STATE_RELEASED:
            if (is_pressed) {
                if (current_tick - key->LastTick > DEBOUNCE_DELAY_MS) {
                    key->CurrentState = KEY_STATE_JUST_PRESSED;
                    key->LastTick = current_tick;
                }
            }
            break;

        case KEY_STATE_JUST_PRESSED:
            key->CurrentState = KEY_STATE_PRESSED;
            break;

        case KEY_STATE_PRESSED:
            if (!is_pressed) {
                if (current_tick - key->LastTick > DEBOUNCE_DELAY_MS) {
                    key->CurrentState = KEY_STATE_JUST_RELEASED;
                    key->LastTick = current_tick;
                }
            }
            break;

        case KEY_STATE_JUST_RELEASED:
            key->CurrentState = KEY_STATE_RELEASED;
            break;
    }
}

/* 获取当前按键状态 */
Key_State Key_GetState(Key_HandleTypeDef *key) {
    return key->CurrentState;
}
