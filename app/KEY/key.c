// key.c
#include "key.h"
#include "main.h"  // ????HAL_GetTick()???

#define DEBOUNCE_DELAY_MS 50  // ????(??:??)

/* ????? */
void Key_Init(Key_HandleTypeDef *key) {
    key->CurrentState = KEY_STATE_RELEASED;
    key->LastTick = HAL_GetTick();
}

/* ??????(??????,???10ms????) */
void Key_Update(Key_HandleTypeDef *key) {
    uint32_t current_tick = HAL_GetTick();
    
    // ????????
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(key->GPIOx, key->GPIO_Pin);
    uint8_t is_pressed = (key->ActiveMode == KEY_ACTIVE_LOW) ? 
                        (pin_state == GPIO_PIN_RESET) : 
                        (pin_state == GPIO_PIN_SET);

    // ?????
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

/* ?????? */
Key_State Key_GetState(Key_HandleTypeDef *key) {
    return key->CurrentState;
}