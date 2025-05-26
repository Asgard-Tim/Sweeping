#include "hc05.h"
#include <string.h>
#include <math.h>

// 新增控制模式定义
#define CONTROL_MODE_REMOTE  0x01
#define CONTROL_MODE_AUTO    0x02

// 私有变量定义
static UART_HandleTypeDef *hc05_uart = NULL;

// 传输协议参数
#define FRAME_HEADER    0xAA
#define FRAME_END       0x55
#define RX_TIMEOUT_MS   200  // 接收超时时间

// 接收状态机
typedef enum {
    WAIT_HEADER,
    RECEIVE_MODE,
    RECEIVE_LEFT,
    RECEIVE_RIGHT,
    CHECK_SUM,
    WAIT_END
} RxState_t;

static uint8_t current_mode = CONTROL_MODE_REMOTE; // 当前控制模式
static RxState_t rx_state = 0;
static uint8_t rx_buffer[2];       // 缓冲区调整为控制指令需要的大小
static uint8_t data_index = 0;
static uint8_t checksum = 0;
static uint8_t current_cmd = 0;    // 当前指令类型
static uint8_t expected_len = 0;   // 预期数据长度
static uint32_t last_rx_time = 0;
static uint8_t hc05_rx_byte = 0;
int8_t hc05_speed_left = 0;
int8_t hc05_speed_right = 0;
uint8_t hc05_warning_flag = 0;

void HC05_Init(UART_HandleTypeDef *huart) {
    hc05_uart = huart;
    HAL_UART_Receive_IT(hc05_uart, &hc05_rx_byte, 1);
}

// 数据打包发送函数
void HC05_SendData(float x, float y, float yaw_deg, 
    float left, float right, float battery, float odo_cm)
{
		// 数据限幅（调整为 int16_t 的合法范围）
        x = x * 10 + 0.5f;
        y = y * 10 + 0.5f;
        x = (x < -32768) ? -32768 : ((x > 32767) ? 32767 : x);
		y = (y < -32768) ? -32768 : ((y > 32767) ? 32767 : y);
		left = (left < -100) ? -100 : ((left > 100) ? 100 : left);
		right = (right < -100) ? -100 : ((right > 100) ? 100 : right);

		// 强制转换为正确的有符号类型
		int16_t x_int = (int16_t)x;
		int16_t y_int = (int16_t)y;
		int8_t left_int = (int8_t)left;
		int8_t right_int = (int8_t)right;
        int8_t battery_int = (int8_t)battery;
		int16_t odo_cm_int = (int16_t)odo_cm;

		// 角度规范化（四舍五入）
		while (yaw_deg < 0) yaw_deg += 360;
		while (yaw_deg >= 360) yaw_deg -= 360;
		uint16_t yaw_10x = (uint16_t)(yaw_deg * 10 + 0.5f); // 四舍五入

		// 构造数据包
		uint8_t tx_buf[1 + sizeof(Robot_Data_t) + 1 + 1] = {0};
		tx_buf[0] = FRAME_HEADER;

		Robot_Data_t *data = (Robot_Data_t*)(tx_buf + 1);
		data->pos_x = x_int;
		data->pos_y = y_int;
		data->yaw = yaw_10x;
		data->left_speed = left_int;
		data->right_speed = right_int;
        data->battery = battery;
        data->odometer = odo_cm_int;

		// 计算校验和
		uint8_t checksum = 0;
		for(uint8_t i=1; i<=sizeof(Robot_Data_t); i++){
		checksum += tx_buf[i];
		}
		tx_buf[1 + sizeof(Robot_Data_t)] = checksum; // 校验和位置
		tx_buf[1 + sizeof(Robot_Data_t) + 1] = FRAME_END; // 帧尾

		// 发送数据
		HAL_UART_Transmit(hc05_uart, tx_buf, sizeof(tx_buf), HAL_MAX_DELAY);
}

void HC05_UART_RxHandler(uint8_t byte) {
    // static uint8_t rx_buffer[2];
    static uint8_t checksum = 0;
    static RxState_t rx_state = WAIT_HEADER;
    static uint32_t last_header_time = 0;

    // 超时检测（300ms未完成接收则重置）
    if(HAL_GetTick() - last_header_time > RX_TIMEOUT_MS && rx_state != WAIT_HEADER) {
        rx_state = WAIT_HEADER;
    }

    switch(rx_state) {
        case WAIT_HEADER:
            if(byte == FRAME_HEADER) {
                checksum = 0;
                rx_state = RECEIVE_MODE;
                last_header_time = HAL_GetTick();
            // } else {
            //     // 非协议数据回传
            //     HAL_UART_Transmit(hc05_uart, &byte, 1, 10);
            }
            break;

        case RECEIVE_MODE:
            current_mode = byte;
            checksum += byte;
            rx_state = RECEIVE_LEFT;
            break;            

        case RECEIVE_LEFT:
            rx_buffer[0] = byte;
            checksum += byte;
            rx_state = RECEIVE_RIGHT;
            break;

        case RECEIVE_RIGHT:
            rx_buffer[1] = byte;
            checksum += byte;
            rx_state = CHECK_SUM;
            break;

        case CHECK_SUM:
            if(byte == (checksum & 0xFF)) {
                rx_state = WAIT_END;
            } else {
                rx_state = WAIT_HEADER; // 校验失败
            }
            break;

        case WAIT_END:
            rx_state = WAIT_HEADER;  // 无论是否成功都重置
            if(byte == FRAME_END) {
                // 仅遥控模式处理速度值
                if(current_mode == CONTROL_MODE_REMOTE) {
                    hc05_speed_left = (int8_t)rx_buffer[0];
                    hc05_speed_right = (int8_t)rx_buffer[1];
                    hc05_warning_flag = 1;

                                        // 停止状态检测
                    if(hc05_speed_left == 0 && hc05_speed_right == 0){
                        hc05_warning_flag = 2;
                    }
                }
            }
            break;
    }
		
    HAL_UART_Receive_IT(hc05_uart, &hc05_rx_byte, 1); // 继续接收
}

void HC05_CheckTimeout(void) {
    if(HAL_GetTick() - last_rx_time > RX_TIMEOUT_MS) {
        rx_state = 0;  // 超时重置状态机
    }
}

uint8_t* HC05_GetRxBytePtr(void)
{
    return &hc05_rx_byte;
}

uint8_t HC05_GetControlMode(void) {
    return current_mode;
}