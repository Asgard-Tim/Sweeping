#ifndef __JY901S_H__
#define __JY901S_H__

#include "main.h"

typedef struct {
    float ax, ay, az;         // ���ٶȣ���λ g��
    float gx, gy, gz;         // ���ٶȣ���λ ��/s��
    float roll, pitch, yaw;   // ŷ���ǣ���λ �㣩
} IMU_Data_t;

/**
 * @brief ��ʼ�� JY901S IMU ģ��
 * @param huart ʹ�õ� UART ������� &huart2��
 */
void JY901S_Init(UART_HandleTypeDef *huart);

/**
 * @brief ���жϻص����õ����ݴ�����
 * @param data �յ�������11�ֽ�����֡
 */
void JY901S_UART_RxHandler(uint8_t *data);

/**
 * @brief ��ȡ��ǰ������ IMU ����
 * @return ����ָ�� IMU ���ݽṹ���ָ��
 */
IMU_Data_t* JY901S_GetData(void);

#endif /* __JY901S_H__ */
