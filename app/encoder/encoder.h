// encoder.h
#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx_hal.h"

/**
 * @brief ����������ṹ��
 */
typedef struct {
    TIM_HandleTypeDef *htim;  ///< �����Ķ�ʱ��������� &htim2 �� &htim5��
    int32_t            count; ///< �ۼ���������������ţ������ɸ���
} Encoder_HandleTypeDef;

/**
 * @brief  ��ʼ���������ӿڣ������� MX_TIMx_Init() ֮����ã�
 * @param  enc   ���������ָ��
 * @param  htim  ��Ӧ�Ķ�ʱ�������&htim2 �������֣�&htim5 �������֣�
 */
void Encoder_Init(Encoder_HandleTypeDef *enc, TIM_HandleTypeDef *htim);

/**
 * @brief  ��ȡ�ۼ�������
 * @param  enc   ���������ָ��
 * @return �ۼ�����ֵ
 */
int32_t Encoder_GetCount(Encoder_HandleTypeDef *enc);

/**
 * @brief  �����������Ϊ 0
 * @param  enc   ���������ָ��
 */
void Encoder_Reset(Encoder_HandleTypeDef *enc);

#endif // __ENCODER_H
