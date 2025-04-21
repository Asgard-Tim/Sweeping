/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OUTAIN1_Pin GPIO_PIN_7
#define OUTAIN1_GPIO_Port GPIOE
#define OUTAIN2_Pin GPIO_PIN_8
#define OUTAIN2_GPIO_Port GPIOE
#define OUTBIN1_Pin GPIO_PIN_12
#define OUTBIN1_GPIO_Port GPIOE
#define OUTBIN2_Pin GPIO_PIN_13
#define OUTBIN2_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_15
#define KEY1_GPIO_Port GPIOE
#define KEY0_Pin GPIO_PIN_11
#define KEY0_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_13
#define LED3_GPIO_Port GPIOD
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
// LED0 options
#define LED0_ON()     HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET) // on
#define LED0_OFF()    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET)   // off
#define LED0_TOGGLE() HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin)                // toggle
// LED1 options
#define LED1_ON()     HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET) // on
#define LED1_OFF()    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)   // off
#define LED1_TOGGLE() HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)                // toggle
// LED2 options
#define LED2_ON()     HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET) // on
#define LED2_OFF()    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)   // off
#define LED2_TOGGLE() HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin)                // toggle
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
