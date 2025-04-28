// encoder.c
#include "encoder.h"

/**
 * @brief  ��ʼ����������������ʱ���ı������ӿ�
 */
void Encoder_Init(Encoder_HandleTypeDef *enc, TIM_HandleTypeDef *htim)
{
    enc->htim = htim;
    enc->count = 0;
    // ���� TIM �� TI1 �� TI2 ͨ���Խ��������ģʽ
    HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_1 | TIM_CHANNEL_2);
}

/**
 * @brief  ��ȡ���ۼ��������������������
 */
int32_t Encoder_GetCount(Encoder_HandleTypeDef *enc)
{
    int16_t raw = __HAL_TIM_GET_COUNTER(enc->htim);
    // �������ϴζ�ȡ�Ĳ�ֵ������ 16 λ���������
    int32_t delta = (int32_t)raw - (int32_t)(enc->count & 0xFFFF);
    if (delta >  32767) delta -= 65536;
    if (delta < -32768) delta += 65536;
    enc->count += delta;
    return enc->count;
}

/**
 * @brief  �����������������ۼ�ֵ������
 */
void Encoder_Reset(Encoder_HandleTypeDef *enc)
{
    __HAL_TIM_SET_COUNTER(enc->htim, 0);
    enc->count = 0;
}
