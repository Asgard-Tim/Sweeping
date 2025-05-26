#include "cliff_sensor.h"
#include "main.h"          // �ⲿhadc1��htim3�������
#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

// ADC DMA�������棬�������ͨ������ֵ��ǰ4�������´����������1����BT15V��ѹͨ��
static uint16_t adc_values[TOTAL_ADC_CHANNELS] = {0};

uint16_t cliff_thresholds[CLIFF_SENSOR_COUNT] = {2000, 2000, 2000, 2000};

#define ADC_REF_VOLTAGE 3.3f
#define ADC_RESOLUTION  4096.0f
#define DIVIDER_RATIO   19.0f   // ��ѹ����
#define MIN_V 12.0f
#define V_Range 4.8f

void CliffSensor_Init(void)
{
    HAL_TIM_Base_Start(&htim3);
    // ����ADC1��ͨ��ɨ��DMA������4���´����� + 1 BT15Vͨ��
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, TOTAL_ADC_CHANNELS);
}

void CliffSensor_GetValues(uint16_t *out_values)
{
    for (int i = 0; i < CLIFF_SENSOR_COUNT; i++)
        out_values[i] = adc_values[i];
}

bool CliffSensor_IsCliff(uint8_t sensor_index)
{
    if (sensor_index >= CLIFF_SENSOR_COUNT) return false;
    return (adc_values[sensor_index] < cliff_thresholds[sensor_index]);
}

uint8_t CliffSensor_GetMask(void)
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < CLIFF_SENSOR_COUNT; i++)
    {
        if (CliffSensor_IsCliff(i))
            mask |= (1U << i);
    }
    return mask;
}

float BT15V_GetVoltage(void)
{
    uint16_t raw = adc_values[CLIFF_SENSOR_COUNT]; // BT15Vͨ������ֵ�����һ����
    float v_adc = ((float)raw) * ADC_REF_VOLTAGE / ADC_RESOLUTION;
    float v_battery = v_adc * DIVIDER_RATIO;
    float battery = (v_battery - MIN_V) / V_Range * 100;
		if (battery > 100)
			return 100;
		return (v_battery - MIN_V) / V_Range * 100;
}

// ��ѡ��ADCת����ɻص���DMAģʽ����HAL�Զ����ã�
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == hadc1.Instance)
    {
        // �������������ݴ���״̬���µ��߼�
    }
}
