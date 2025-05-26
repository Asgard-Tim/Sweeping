#ifndef CLIFF_SENSOR_H
#define CLIFF_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#define CLIFF_SENSOR_COUNT 4
#define BT15V_SENSOR_COUNT 1
#define TOTAL_ADC_CHANNELS (CLIFF_SENSOR_COUNT + BT15V_SENSOR_COUNT)

// ���´��������붨��
#define CLIFF_1   (1U << 0)
#define CLIFF_2   (1U << 1)
#define CLIFF_3   (1U << 2)
#define CLIFF_4   (1U << 3)

extern uint16_t cliff_thresholds[CLIFF_SENSOR_COUNT];

/**
 * @brief ��ʼ�����´�������BT15V��ѹ����������ADC+DMA
 */
void CliffSensor_Init(void);

/**
 * @brief ��ȡ��ǰ���´�����ADCֵ��4����
 */
void CliffSensor_GetValues(uint16_t *out_values);

/**
 * @brief �ж�ĳ�����´������Ƿ��⵽����
 */
bool CliffSensor_IsCliff(uint8_t sensor_index);

/**
 * @brief ��ȡ�������´��������������
 */
uint8_t CliffSensor_GetMask(void);

/**
 * @brief ��ȡBT15V��ѹ����ֵ����λ�����أ�
 */
float BT15V_GetVoltage(void);

#endif // CLIFF_SENSOR_H
