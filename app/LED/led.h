// led.h
#ifndef __LED_H
#define __LED_H

#include "stm32f4xx_hal.h"  // ??HAL????

/* LED???? */
typedef enum {
  LED_ACTIVE_LOW = 0,  // ?????(???VCC)
  LED_ACTIVE_HIGH      // ?????(???GND)
} LED_ActiveMode;

/* LED????? */
typedef struct {
  GPIO_TypeDef *GPIOx;      // GPIO??(?GPIOE)
  uint16_t GPIO_Pin;        // ???(?GPIO_PIN_14)
  LED_ActiveMode ActiveMode; // ????
} LED_HandleTypeDef;

/* ???? */
void LED_Init(LED_HandleTypeDef *led);
void LED_On(LED_HandleTypeDef *led);
void LED_Off(LED_HandleTypeDef *led);
void LED_Toggle(LED_HandleTypeDef *led);

#endif /* __LED_H */