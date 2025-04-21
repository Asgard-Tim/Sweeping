#include "jy901s.h"

#define FRAME_LEN 11  // JY901S ??????? 11 ??

static UART_HandleTypeDef *jy_uart = NULL;
static IMU_Data_t imu_data;

void JY901S_Init(UART_HandleTypeDef *huart)
{
    jy_uart = huart;
    static uint8_t rx_buf[FRAME_LEN];
    HAL_UART_Receive_IT(jy_uart, rx_buf, FRAME_LEN); // ??????
}

void JY901S_UART_RxHandler(uint8_t *data)
{
    // ????(???????????????)
    if (data[0] == 0x55) {
        switch (data[1]) {
            case 0x51: // ???
                imu_data.ax = (short)(data[3] << 8 | data[2]) / 32768.0f * 16;
                imu_data.ay = (short)(data[5] << 8 | data[4]) / 32768.0f * 16;
                imu_data.az = (short)(data[7] << 8 | data[6]) / 32768.0f * 16;
                break;
            case 0x52: // ???
                imu_data.gx = (short)(data[3] << 8 | data[2]) / 32768.0f * 2000;
                imu_data.gy = (short)(data[5] << 8 | data[4]) / 32768.0f * 2000;
                imu_data.gz = (short)(data[7] << 8 | data[6]) / 32768.0f * 2000;
                break;
            case 0x53: // ???
                imu_data.roll  = (short)(data[3] << 8 | data[2]) / 32768.0f * 180;
                imu_data.pitch = (short)(data[5] << 8 | data[4]) / 32768.0f * 180;
                imu_data.yaw   = (short)(data[7] << 8 | data[6]) / 32768.0f * 180;
                break;
            default:
                break;
        }
    }

    // ?????????
    HAL_UART_Receive_IT(jy_uart, data, FRAME_LEN);
}

IMU_Data_t* JY901S_GetData(void)
{
    return &imu_data;
}
