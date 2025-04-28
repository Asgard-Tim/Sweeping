// encoder.c
#include "encoder.h"

/**
 * @brief  初始化编码器：启动定时器的编码器接口
 */
void Encoder_Init(Encoder_HandleTypeDef *enc, TIM_HandleTypeDef *htim)
{
    enc->htim = htim;
    enc->count = 0;
    // 启动 TIM 的 TI1 和 TI2 通道以进入编码器模式
    HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_1 | TIM_CHANNEL_2);
}

/**
 * @brief  读取并累计脉冲数，处理上下溢出
 */
int32_t Encoder_GetCount(Encoder_HandleTypeDef *enc)
{
    int16_t raw = __HAL_TIM_GET_COUNTER(enc->htim);
    // 计算与上次读取的差值（考虑 16 位上下溢出）
    int32_t delta = (int32_t)raw - (int32_t)(enc->count & 0xFFFF);
    if (delta >  32767) delta -= 65536;
    if (delta < -32768) delta += 65536;
    enc->count += delta;
    return enc->count;
}

/**
 * @brief  将编码器计数器和累计值都清零
 */
void Encoder_Reset(Encoder_HandleTypeDef *enc)
{
    __HAL_TIM_SET_COUNTER(enc->htim, 0);
    enc->count = 0;
}
