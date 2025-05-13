#include "hc05.h"
#include <string.h>
#include <math.h>

// 私有变量定义
static UART_HandleTypeDef *hc05_uart = NULL;
static uint8_t hc05_rx_byte = 0;

// 传输协议参数
#define FRAME_HEADER    0xAA
#define FRAME_END       0x55
#define CONTROL_CMD_ID  0x01  // 新增控制指令标识

// 接收状态机
typedef enum {
    WAIT_HEADER,
    RECEIVE_CMD,      // 新增状态：接收指令类型
    RECEIVE_DATA,
    CHECK_SUM,
    WAIT_END
} RxState_t;

static RxState_t rx_state = WAIT_HEADER;
static uint8_t rx_buffer[4];       // 缓冲区调整为控制指令需要的大小
static uint8_t data_index = 0;
static uint8_t checksum = 0;
static uint8_t current_cmd = 0;    // 当前指令类型
static uint8_t expected_len = 0;   // 预期数据长度

void HC05_Init(UART_HandleTypeDef *huart) {
    hc05_uart = huart;
    HAL_UART_Receive_IT(hc05_uart, &hc05_rx_byte, 1);
}

// 数据打包发送函数
void HC05_SendData(float x, float y, float yaw_deg, float left, float right)
{
		// 数据限幅（调整为 int16_t 的合法范围）
		x = (x < -32768) ? -32768 : ((x > 32767) ? 32767 : x);
		y = (y < -32768) ? -32768 : ((y > 32767) ? 32767 : y);
		left = (left < -100) ? -100 : ((left > 100) ? 100 : left);
		right = (right < -100) ? -100 : ((right > 100) ? 100 : right);

		// 强制转换为正确的有符号类型
		int16_t x_int = (int16_t)x;
		int16_t y_int = (int16_t)y;
		int8_t left_int = (int8_t)left;
		int8_t right_int = (int8_t)right;

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
    switch(rx_state) {
    case WAIT_HEADER:
        if(byte == FRAME_HEADER) {
            checksum = 0;
            data_index = 0;
            rx_state = RECEIVE_CMD;  // 进入接收指令类型状态
        } else {
            HC05_OnByteReceived(byte); // 非协议数据转给ASCII处理
        }
        break;

    case RECEIVE_CMD:
        current_cmd = byte;
        checksum += byte;  // 校验和包含指令类型
        switch(current_cmd) {
            case CONTROL_CMD_ID:
                expected_len = 2;  // 控制指令数据长度2字节
                rx_state = RECEIVE_DATA;
                break;
            default:              // 未知指令重置状态机
                rx_state = WAIT_HEADER;
        }
        break;

    case RECEIVE_DATA:
        if(data_index < expected_len) {
            rx_buffer[data_index++] = byte;
            checksum += byte;
        } else {
            rx_state = CHECK_SUM;
        }
        break;

    case CHECK_SUM:
        if(byte == (checksum & 0xFF)) {
            rx_state = WAIT_END;
        } else {
            rx_state = WAIT_HEADER; // 校验失败
        }
        break;

    case WAIT_END:
        if(byte == FRAME_END) {
            // 完整帧处理
            if(current_cmd == CONTROL_CMD_ID) {
                int8_t speed_l = (int8_t)rx_buffer[0];
                int8_t speed_r = (int8_t)rx_buffer[1];
                Motor_SetSpeed(speed_l, speed_r); // 电机控制函数需自行实现
            }
        }
        rx_state = WAIT_HEADER;
        break;
    }

    // 继续接收
    HAL_UART_Receive_IT(hc05_uart, &hc05_rx_byte, 1);
}

uint8_t* HC05_GetRxBytePtr(void)
{
    return &hc05_rx_byte;
}