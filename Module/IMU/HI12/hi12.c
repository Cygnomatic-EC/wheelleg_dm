#include "hi12.h"
#include <string.h>
#include "crc.h"

static hi12_t hi12_instance;

void imuCallback(uint8_t* data, uint16_t len);

void HI12_Init(UART_HandleTypeDef* uart_handle)
{
    if (uart_handle == NULL)
        return;
    BSP_UART_Init(&hi12_instance.imu_uart, uart_handle, 256000, imuCallback, NULL, 0, RX_BUFFER_SIZE);
    hi12_instance.init = 1;
}

void imuCallback(uint8_t* data, const uint16_t len)
{
    if(len <82)
        return;
    for(int i=0;i<len;i++)
    {
        if(data[i]==0x5A&&data[i+1]==0xA5)
        {
            const uint16_t data_len = (uint16_t)data[i+3]<<8 | data[i+2];
            uint16_t crc = 0;

            crc16_update(&crc, &data[i], 4);
            crc16_update(&crc, &data[i] + 6, data_len);

            if(crc == ((uint16_t)data[i+5]<<8 | data[i+4]))
            {
                memcpy(&hi12_instance.imu_data, &data[i], sizeof(SensorDataPacket));
                break;
            }
        }
    }
}

hi12_t *Get_HI12_Instance(void)
{
    return &hi12_instance;
}
