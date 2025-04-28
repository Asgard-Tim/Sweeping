#ifndef __CRASH_SENSOR_H
#define __CRASH_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"  // 包含 GPIO 针脚定义（由 CubeMX 生成）

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化碰撞传感器接口
 * @note   基于 CubeMX 已生成的 GPIO 初始化，若需额外配置可在此完善
 */
void CrashSensor_Init(void);

/**
 * @brief  获取左侧碰撞传感器状态
 * @retval true  检测到碰撞（开关闭合）
 *         false 未检测到碰撞
 */
bool CrashSensor_Left(void);

/**
 * @brief  获取右侧碰撞传感器状态
 * @retval true  检测到碰撞（开关闭合）
 *         false 未检测到碰撞
 */
bool CrashSensor_Right(void);

/**
 * @brief  获取两侧碰撞传感器整体状态
 * @retval bit0: 左侧状态 (1=碰撞)
 *         bit1: 右侧状态 (1=碰撞)
 */
uint8_t CrashSensor_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif // __CRASH_SENSOR_H
