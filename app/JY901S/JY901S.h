#ifndef __JY901S_H__
#define __JY901S_H__

#include "main.h"

typedef struct {
    float ax, ay, az;         // 加速度（单位 g）
    float gx, gy, gz;         // 角速度（单位 °/s）
    float roll, pitch, yaw;   // 欧拉角（单位 °）
} IMU_Data_t;

/**
 * @brief 初始化 JY901S IMU 模块
 * @param huart 使用的 UART 句柄（如 &huart2）
 */
void JY901S_Init(UART_HandleTypeDef *huart);

/**
 * @brief 供中断回调调用的数据处理函数
 * @param data 收到的完整11字节数据帧
 */
void JY901S_UART_RxHandler(uint8_t *data);

/**
 * @brief 获取当前解析的 IMU 数据
 * @return 返回指向 IMU 数据结构体的指针
 */
IMU_Data_t* JY901S_GetData(void);

#endif /* __JY901S_H__ */
