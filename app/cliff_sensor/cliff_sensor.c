#include "cliff_sensor.h"
#include "main.h"          // �ṩ hadc1��htim3 ������
#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

static uint16_t adc_values[CLIFF_SENSOR_COUNT] = {0};  // �洢 ADC ת�����
uint16_t cliff_thresholds[CLIFF_SENSOR_COUNT] = {2000, 2000, 2000, 2000};  // ÿ�����������ж���ֵ

/**
 * @brief ���´�������ʼ����������ʱ���� ADC-DMA ģʽ
 */
void CliffSensor_Init(void)
{
    HAL_TIM_Base_Start(&htim3);  // ���� TIM3 ������ʱ��
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, CLIFF_SENSOR_COUNT);  // ���� ADC + DMA����ȡ CLIFF_SENSOR_COUNT ��ͨ��
}

/**
 * @brief ��ȡ��ǰ���д������� ADC ��ֵ
 * @param out �ⲿ����ָ�룬����ÿ����������ǰ�� ADC ���
 */
void CliffSensor_GetValues(uint16_t *out)
{
    for (int i = 0; i < CLIFF_SENSOR_COUNT; i++)
        out[i] = adc_values[i];
}

/**
 * @brief �жϵ����������Ƿ��⵽���£�ADC ֵ������ֵ��
 * @param idx ��������ţ�0~3��
 * @retval true ��ʾ��⵽���£�false ��ʾ����
 */
bool CliffSensor_IsCliff(uint8_t idx)
{
    if (idx >= CLIFF_SENSOR_COUNT) return false;  // Խ�籣��
    return (adc_values[idx] < cliff_thresholds[idx]);
}

/**
 * @brief ������д�������״̬���������뷽ʽ����
 * @retval ����ֵ��һ��λ���룬��ЩλΪ1��ʾ��Ӧ��������⵽����
 */
uint8_t CliffSensor_GetMask(void)
{
    uint8_t mask = 0;
    if (CliffSensor_IsCliff(0)) mask |= CLIFF_1;
    if (CliffSensor_IsCliff(1)) mask |= CLIFF_2;
    if (CliffSensor_IsCliff(2)) mask |= CLIFF_3;
    if (CliffSensor_IsCliff(3)) mask |= CLIFF_4;
    return mask;
}

/**
 * @brief ADC ת����ɻص�������DMAģʽ���� HAL �Զ����ã�
 * @param hadc ��ǰ�����ص��� ADC ���
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == hadc1.Instance)
    {
        // �˴����Դ��� ADC �����Ѹ��º���߼�����������һ����־λ
        // ���磺cliff_data_ready = true;
    }
}
