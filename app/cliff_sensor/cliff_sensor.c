#include "cliff_sensor.h"
#include "main.h"          // 提供 hadc1、htim3 的声明
#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

static uint16_t adc_values[CLIFF_SENSOR_COUNT] = {0};  // 存储 ADC 转换结果
uint16_t cliff_thresholds[CLIFF_SENSOR_COUNT] = {2000, 2000, 2000, 2000};  // 每个传感器的判断阈值

/**
 * @brief 悬崖传感器初始化：启动定时器和 ADC-DMA 模式
 */
void CliffSensor_Init(void)
{
    HAL_TIM_Base_Start(&htim3);  // 启动 TIM3 基本定时器
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, CLIFF_SENSOR_COUNT);  // 启动 ADC + DMA，读取 CLIFF_SENSOR_COUNT 个通道
}

/**
 * @brief 获取当前所有传感器的 ADC 数值
 * @param out 外部数组指针，填入每个传感器当前的 ADC 结果
 */
void CliffSensor_GetValues(uint16_t *out)
{
    for (int i = 0; i < CLIFF_SENSOR_COUNT; i++)
        out[i] = adc_values[i];
}

/**
 * @brief 判断单个传感器是否检测到悬崖（ADC 值低于阈值）
 * @param idx 传感器编号（0~3）
 * @retval true 表示检测到悬崖，false 表示正常
 */
bool CliffSensor_IsCliff(uint8_t idx)
{
    if (idx >= CLIFF_SENSOR_COUNT) return false;  // 越界保护
    return (adc_values[idx] < cliff_thresholds[idx]);
}

/**
 * @brief 检测所有传感器的状态，并以掩码方式返回
 * @retval 返回值是一个位掩码，哪些位为1表示对应传感器检测到悬崖
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
 * @brief ADC 转换完成回调函数（DMA模式下由 HAL 自动调用）
 * @param hadc 当前触发回调的 ADC 句柄
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == hadc1.Instance)
    {
        // 此处可以处理 ADC 数据已更新后的逻辑，比如设置一个标志位
        // 例如：cliff_data_ready = true;
    }
}
