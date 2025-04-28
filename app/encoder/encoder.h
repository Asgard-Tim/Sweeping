// encoder.h
#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx_hal.h"

/**
 * @brief 编码器句柄结构体
 */
typedef struct {
    TIM_HandleTypeDef *htim;  ///< 关联的定时器句柄（如 &htim2 或 &htim5）
    int32_t            count; ///< 累计脉冲计数（带符号，可正可负）
} Encoder_HandleTypeDef;

/**
 * @brief  初始化编码器接口（必须在 MX_TIMx_Init() 之后调用）
 * @param  enc   编码器句柄指针
 * @param  htim  对应的定时器句柄（&htim2 用于左轮，&htim5 用于右轮）
 */
void Encoder_Init(Encoder_HandleTypeDef *enc, TIM_HandleTypeDef *htim);

/**
 * @brief  获取累计脉冲数
 * @param  enc   编码器句柄指针
 * @return 累计脉冲值
 */
int32_t Encoder_GetCount(Encoder_HandleTypeDef *enc);

/**
 * @brief  重置脉冲计数为 0
 * @param  enc   编码器句柄指针
 */
void Encoder_Reset(Encoder_HandleTypeDef *enc);

#endif // __ENCODER_H
