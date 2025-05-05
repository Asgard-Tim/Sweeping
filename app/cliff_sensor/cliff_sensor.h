#ifndef CLIFF_SENSOR_H
#define CLIFF_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#define CLIFF_SENSOR_COUNT 4  // 悬崖传感器数量

// 悬崖传感器掩码定义（用于按位标识传感器状态）
#define CLIFF_1   (1U << 0)   // 第1个传感器
#define CLIFF_2   (1U << 1)   // 第2个传感器
#define CLIFF_3   (1U << 2)   // 第3个传感器
#define CLIFF_4   (1U << 3)   // 第4个传感器

// 悬崖传感器的触发阈值数组（由 .c 文件中定义）
extern uint16_t cliff_thresholds[CLIFF_SENSOR_COUNT];

/**
 * @brief 悬崖传感器初始化函数，配置 TIM3 和 ADC + DMA
 */
void CliffSensor_Init(void);

/**
 * @brief  获取当前所有悬崖传感器的 ADC 数值
 * @param  out_values 指向外部数组的指针，存放每个传感器的 ADC 读取值（4个）
 */
void CliffSensor_GetValues(uint16_t *out_values);

/**
 * @brief  判断指定编号的传感器是否检测到悬崖
 * @param  sensor_index 传感器索引（0~3）
 * @retval true 表示检测到悬崖，false 表示正常
 */
bool CliffSensor_IsCliff(uint8_t sensor_index);

/**
 * @brief  获取当前所有检测到悬崖的传感器掩码
 * @retval 0 表示未检测到悬崖  
 *         例如返回 CLIFF_1，表示第1个传感器触发；  
 *         返回 CLIFF_1 | CLIFF_2 表示多个传感器同时触发
 */
uint8_t CliffSensor_GetMask(void);

#endif // CLIFF_SENSOR_H
