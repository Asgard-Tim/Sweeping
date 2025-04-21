#include "cliff_sensor.h"
#include "main.h"          // for hadc1, htim3
#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

static uint16_t adc_values[CLIFF_SENSOR_COUNT] = {0};
uint16_t cliff_thresholds[CLIFF_SENSOR_COUNT] = {2000,2000,2000,2000};

void CliffSensor_Init(void)
{
    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, CLIFF_SENSOR_COUNT);
}

void CliffSensor_GetValues(uint16_t *out)
{
    for (int i = 0; i < CLIFF_SENSOR_COUNT; i++)
        out[i] = adc_values[i];
}

bool CliffSensor_IsCliff(uint8_t idx)
{
    if (idx >= CLIFF_SENSOR_COUNT) return false;
    return (adc_values[idx] < cliff_thresholds[idx]);
}

uint8_t CliffSensor_GetMask(void)
{
    uint8_t mask = 0;
    if (CliffSensor_IsCliff(0)) mask |= CLIFF_1;
    if (CliffSensor_IsCliff(1)) mask |= CLIFF_2;
    if (CliffSensor_IsCliff(2)) mask |= CLIFF_3;
    if (CliffSensor_IsCliff(3)) mask |= CLIFF_4;
    return mask;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == hadc1.Instance)
    {
        // ??????????????????
    }
}
