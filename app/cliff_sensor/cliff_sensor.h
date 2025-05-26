#ifndef CLIFF_SENSOR_H
#define CLIFF_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#define CLIFF_SENSOR_COUNT 4
#define BT15V_SENSOR_COUNT 1
#define TOTAL_ADC_CHANNELS (CLIFF_SENSOR_COUNT + BT15V_SENSOR_COUNT)

// 悬崖传感器掩码定义
#define CLIFF_1   (1U << 0)
#define CLIFF_2   (1U << 1)
#define CLIFF_3   (1U << 2)
#define CLIFF_4   (1U << 3)

extern uint16_t cliff_thresholds[CLIFF_SENSOR_COUNT];

/**
 * @brief 初始化悬崖传感器和BT15V电压采样，启动ADC+DMA
 */
void CliffSensor_Init(void);

/**
 * @brief 获取当前悬崖传感器ADC值（4个）
 */
void CliffSensor_GetValues(uint16_t *out_values);

/**
 * @brief 判断某个悬崖传感器是否检测到悬崖
 */
bool CliffSensor_IsCliff(uint8_t sensor_index);

/**
 * @brief 获取所有悬崖传感器检测结果掩码
 */
uint8_t CliffSensor_GetMask(void);

/**
 * @brief 获取BT15V电压采样值（单位：伏特）
 */
float BT15V_GetVoltage(void);

#endif // CLIFF_SENSOR_H
