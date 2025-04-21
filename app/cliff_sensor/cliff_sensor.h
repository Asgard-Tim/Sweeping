#ifndef CLIFF_SENSOR_H
#define CLIFF_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#define CLIFF_SENSOR_COUNT 4

// ?????????
#define CLIFF_1   (1U << 0)
#define CLIFF_2  (1U << 1)
#define CLIFF_3  (1U << 2)
#define CLIFF_4   (1U << 3)

// ??????(extern ??? .c)
extern uint16_t cliff_thresholds[CLIFF_SENSOR_COUNT];

/**
 * @brief ?????,?? TIM3 ? ADC+DMA
 */
void CliffSensor_Init(void);

/**
 * @brief  ?????? ADC ?
 */
void CliffSensor_GetValues(uint16_t *out_values);

/**
 * @brief  ??????????
 */
bool CliffSensor_IsCliff(uint8_t sensor_index);

/**
 * @brief  ?????????????
 * @retval 0               = ???????  
 *         CLIFF_LEFT     = ?????  
 *         CLIFF_LEFT|CLIFF_FRONT = ?+?????,??  
 */
uint8_t CliffSensor_GetMask(void);

#endif // CLIFF_SENSOR_H
