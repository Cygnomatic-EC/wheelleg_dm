#include "dbus.h"
#include "usart.h"

__attribute__((section(".dma_buffer"), aligned(32))) static rc_instance dbus_instance;

void RemoteDataProcess_UART(uint8_t *data, uint16_t len);
void RemoteDataProcess_CAN(const uint8_t *data, uint32_t id, void* arg);
void dbus_init(const RC_MODE mode, FDCAN_HandleTypeDef* hcan)
{
    memset(&dbus_instance, 0, sizeof(rc_instance));
    rc_instance* rc_ins = &dbus_instance;
    memset(&rc_ins->rc_data, 0, sizeof(rc_data_t));
    if (mode == RC_DIRECT)
        rc_ins->rc_data.mode = RC_DIRECT;
    else if (mode == RC_CAN)
        rc_ins->rc_data.mode = RC_CAN;
    rc_ins->dbus_can = BSP_CAN_Init(hcan);
    if (rc_ins->rc_data.mode == RC_DIRECT)
        BSP_UART_Init(&rc_ins->dbus_usart, &DBUS_HUART, 100000, RemoteDataProcess_UART, NULL, 0, DBUS_MAX_LEN);
    else
        BSP_CAN_RegisterStdCallback(rc_ins->dbus_can, DBUS_CANID, RemoteDataProcess_CAN, &rc_ins->rc_data);
    dbus_instance.init = 1;
}

void RemoteDataProcess_UART(uint8_t *data, const uint16_t len)
{
    rc_data_t* rc_data = &dbus_instance.rc_data;
    if (data == NULL || len != 18)
    {
        return;
    }
    rc_data->ch0 = (int16_t)(((int16_t)data[0] | ((int16_t)data[1] << 8)) & 0x07FF);
    rc_data->ch1 = (int16_t)((((int16_t)data[1] >> 3) | ((int16_t)data[2] << 5)) & 0x07FF);
    rc_data->ch2 = (int16_t)((((int16_t)data[2] >> 6) | ((int16_t)data[3] << 2) | ((int16_t)data[4] << 10)) & 0x07FF);
    rc_data->ch3 = (int16_t)((((int16_t)data[4] >> 1) | ((int16_t)data[5] << 7)) & 0x07FF);
    rc_data->roll = (int16_t)(data[16] | (data[17] << 8));
    rc_data->ch0 -= RC_CH_VALUE_OFFSET;
    rc_data->ch1 -= RC_CH_VALUE_OFFSET;
    rc_data->ch2 -= RC_CH_VALUE_OFFSET;
    rc_data->ch3 -= RC_CH_VALUE_OFFSET;
    rc_data->roll -= RC_CH_VALUE_OFFSET;
    rc_data->s1 = ((data[5] >> 4) & 0x0003);
    rc_data->s2 = ((data[5] >> 4) & 0x000C) >> 2;


}

void RemoteDataProcess_CAN(const uint8_t *data, uint32_t id, void* arg)
{
    if (data == NULL || arg == NULL)
    {
        return;
    }
    rc_data_t* rc_data = (rc_data_t*)arg;

    rc_data->ch0 = (int16_t)(((int16_t)data[0] | ((int16_t)data[1] << 8)) & 0x07FF);
    rc_data->ch1 = (int16_t)((((int16_t)data[1] >> 3) | ((int16_t)data[2] << 5)) & 0x07FF);
    rc_data->ch2 = (int16_t)((((int16_t)data[2] >> 6) | ((int16_t)data[3] << 2) | ((int16_t)data[4] << 10)) & 0x07FF);
    rc_data->ch3 = (int16_t)((((int16_t)data[4] >> 1) | ((int16_t)data[5] << 7)) & 0x07FF);
    rc_data->roll = (int16_t)(data[6] | (data[7] << 8));
    rc_data->ch0 -= RC_CH_VALUE_OFFSET;
    rc_data->ch1 -= RC_CH_VALUE_OFFSET;
    rc_data->ch2 -= RC_CH_VALUE_OFFSET;
    rc_data->ch3 -= RC_CH_VALUE_OFFSET;
    rc_data->roll -= RC_CH_VALUE_OFFSET;
    rc_data->s1 = ((data[5] >> 4) & 0x0003);
    rc_data->s2 = ((data[5] >> 4) & 0x000C) >> 2;
}

void RemoteData_UART2CAN()
{
    if (!dbus_instance.init)
        return;
    uint8_t data[8];

    const int16_t ch0 = (int16_t)(dbus_instance.rc_data.ch0 + 1024);
    const int16_t ch1 = (int16_t)(dbus_instance.rc_data.ch1 + 1024);
    const int16_t ch2 = (int16_t)(dbus_instance.rc_data.ch2 + 1024);
    const int16_t ch3 = (int16_t)(dbus_instance.rc_data.ch3 + 1024);

    data[0] = ch0 & 0xFF;
    data[1] = ((ch0 >> 8) & 0x07) | ((ch1 << 3) & 0xF8);
    data[2] = ((ch1 >> 5) & 0x3F) | ((ch2 << 6) & 0xC0);
    data[3] = (ch2 >> 2) & 0xFF;
    data[4] = ((ch2 >> 10) & 0x01) | ((ch3 << 1) & 0xFE);
    data[5] = ((ch3 >> 7) & 0x0F) | ((dbus_instance.rc_data.s1 & 0x03) << 4) | ((dbus_instance.rc_data.s2 & 0x03) << 6);
    data[6] = dbus_instance.rc_data.roll & 0xFF;
    data[7] = (dbus_instance.rc_data.roll >> 8) & 0xFF;

    BSP_CAN_Transmit(dbus_instance.dbus_can, DBUS_CANID, FDCAN_STANDARD_ID, data, 8);
}

rc_instance *Get_DBUS_Ptr(void)
{
    return &dbus_instance;
}
