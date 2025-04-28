// key.h
#ifndef __KEY_H
#define __KEY_H

#include "stm32f4xx_hal.h"

/* ����������ƽģʽ */
typedef enum {
    KEY_ACTIVE_LOW = 0,  // �͵�ƽ��Ч�����½�GND��
    KEY_ACTIVE_HIGH      // �ߵ�ƽ��Ч�����½�VCC��
} Key_ActiveMode;

/* ����״̬ */
typedef enum {
    KEY_STATE_RELEASED = 0,      // �ɿ�״̬
    KEY_STATE_PRESSED,           // ����״̬
    KEY_STATE_JUST_PRESSED,      // �ոհ��£����أ�
    KEY_STATE_JUST_RELEASED      // �ո��ɿ������أ�
} Key_State;

/* ��������ṹ�� */
typedef struct {
    GPIO_TypeDef *GPIOx;          // GPIO�˿ڣ���GPIOD��
    uint16_t GPIO_Pin;            // GPIO���ţ���GPIO_PIN_11��
    Key_ActiveMode ActiveMode;    // ��Ч��ƽģʽ
    Key_State CurrentState;       // ��ǰ״̬
    uint32_t LastTick;            // ���һ��״̬�仯ʱ��ʱ���
} Key_HandleTypeDef;

/* �������� */
void Key_Init(Key_HandleTypeDef *key);
void Key_Update(Key_HandleTypeDef *key);
Key_State Key_GetState(Key_HandleTypeDef *key);

#endif /* __KEY_H */
