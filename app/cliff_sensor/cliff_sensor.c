#include "cliff_sensor.h"
#include "main.h"          // 外部hadc1、htim3句柄声明
#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

// ADC DMA采样缓存，存放所有通道采样值：前4个是悬崖传感器，最后1个是BT15V电压通道
static uint16_t adc_values[TOTAL_ADC_CHANNELS] = {0};

uint16_t cliff_thresholds[CLIFF_SENSOR_COUNT] = {2000, 2000, 2000, 2000};

#define ADC_REF_VOLTAGE 3.3f
#define ADC_RESOLUTION  4096.0f
#define DIVIDER_RATIO   19.0f   // 分压比例
#define MIN_V 12.0f
#define V_Range 4.8f

void CliffSensor_Init(void)
{
    HAL_TIM_Base_Start(&htim3);
    // 启动ADC1多通道扫描DMA，采样4悬崖传感器 + 1 BT15V通道
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
    uint16_t raw = adc_values[CLIFF_SENSOR_COUNT]; // BT15V通道采样值（最后一个）
    float v_adc = ((float)raw) * ADC_REF_VOLTAGE / ADC_RESOLUTION;
    float v_battery = v_adc * DIVIDER_RATIO;
    float battery = (v_battery - MIN_V) / V_Range * 100;
		if (battery > 100)
			return 100;
		return (v_battery - MIN_V) / V_Range * 100;
}

// 可选：ADC转换完成回调（DMA模式下由HAL自动调用）
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == hadc1.Instance)
    {
        // 这里可以添加数据处理、状态更新等逻辑
    }
}
