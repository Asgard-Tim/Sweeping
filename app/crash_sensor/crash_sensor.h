#ifndef __CRASH_SENSOR_H
#define __CRASH_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"  // ���� GPIO ��Ŷ��壨�� CubeMX ���ɣ�

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  ��ʼ����ײ�������ӿ�
 * @note   ���� CubeMX �����ɵ� GPIO ��ʼ��������������ÿ��ڴ�����
 */
void CrashSensor_Init(void);

/**
 * @brief  ��ȡ�����ײ������״̬
 * @retval true  ��⵽��ײ�����رպϣ�
 *         false δ��⵽��ײ
 */
bool CrashSensor_Left(void);

/**
 * @brief  ��ȡ�Ҳ���ײ������״̬
 * @retval true  ��⵽��ײ�����رպϣ�
 *         false δ��⵽��ײ
 */
bool CrashSensor_Right(void);

/**
 * @brief  ��ȡ������ײ����������״̬
 * @retval bit0: ���״̬ (1=��ײ)
 *         bit1: �Ҳ�״̬ (1=��ײ)
 */
uint8_t CrashSensor_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif // __CRASH_SENSOR_H
