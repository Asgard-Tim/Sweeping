#ifndef __HC05_H
#define __HC05_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "main.h"

// 协议标识定义
#define FRAME_HEADER        0xAA
#define CONTROL_CMD_ID      0x01   // 新增控制指令标识
#define FRAME_END           0x55

// 数据帧结构定义（保持向下兼容）
#pragma pack(push, 1)
typedef struct {
    int16_t pos_x;      // X坐标 (-999~999)
    int16_t pos_y;      // Y坐标 (-999~999)
    uint16_t yaw;       // 转角 (0-3599，实际值*10)
    int8_t  left_speed; // 左轮速度 (-100~100)
    int8_t  right_speed;// 右轮速度 (-100~100)
    uint8_t battery;    // 电量（百分比）
    uint16_t odometer;  // 里程（厘米）
} Robot_Data_t;
#pragma pack(pop)
extern uint8_t hc05_rx_byte;
extern int8_t hc05_speed_left;
extern int8_t hc05_speed_right;
extern uint8_t hc05_warning_flag;

// 电机控制回调声明（用户需在main.c实现）
__weak void Motor_SetSpeed(int8_t left, int8_t right);

/* 蓝牙模块初始化 */
void HC05_Init(UART_HandleTypeDef *huart);

/* 数据发送接口（保持原有功能） */
void HC05_SendData(float x, float y, float yaw_deg, float left, float right, float battery, float odo_cm);

/* 接收中断处理函数 */
void HC05_UART_RxHandler(uint8_t byte);

/* 获取接收字节指针（用于HAL库回调） */
uint8_t* HC05_GetRxBytePtr(void);

void HC05_CheckTimeout(void);

uint8_t HC05_GetControlMode(void);

#endif