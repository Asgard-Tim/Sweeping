#ifndef CLIFF_SENSOR_H
#define CLIFF_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#define CLIFF_SENSOR_COUNT 4  // ���´���������

// ���´��������붨�壨���ڰ�λ��ʶ������״̬��
#define CLIFF_1   (1U << 0)   // ��1��������
#define CLIFF_2   (1U << 1)   // ��2��������
#define CLIFF_3   (1U << 2)   // ��3��������
#define CLIFF_4   (1U << 3)   // ��4��������

// ���´������Ĵ�����ֵ���飨�� .c �ļ��ж��壩
extern uint16_t cliff_thresholds[CLIFF_SENSOR_COUNT];

/**
 * @brief ���´�������ʼ������������ TIM3 �� ADC + DMA
 */
void CliffSensor_Init(void);

/**
 * @brief  ��ȡ��ǰ�������´������� ADC ��ֵ
 * @param  out_values ָ���ⲿ�����ָ�룬���ÿ���������� ADC ��ȡֵ��4����
 */
void CliffSensor_GetValues(uint16_t *out_values);

/**
 * @brief  �ж�ָ����ŵĴ������Ƿ��⵽����
 * @param  sensor_index ������������0~3��
 * @retval true ��ʾ��⵽���£�false ��ʾ����
 */
bool CliffSensor_IsCliff(uint8_t sensor_index);

/**
 * @brief  ��ȡ��ǰ���м�⵽���µĴ���������
 * @retval 0 ��ʾδ��⵽����  
 *         ���緵�� CLIFF_1����ʾ��1��������������  
 *         ���� CLIFF_1 | CLIFF_2 ��ʾ���������ͬʱ����
 */
uint8_t CliffSensor_GetMask(void);

#endif // CLIFF_SENSOR_H
