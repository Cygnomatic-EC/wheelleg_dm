#include "wbus.h"
#include "usart.h"

__attribute__((section(".dma_buffer"), aligned(128))) static wbus_instance wbus_ins;

void WBUSDataProcess_UART(uint8_t *data, uint16_t len);
void WBUSDataProcess_CAN(const uint8_t *data, uint32_t id, void* arg);
void wbus_init(const WBUS_MODE mode, FDCAN_HandleTypeDef* hcan)
{
    memset(&wbus_ins, 0, sizeof(wbus_ins));
    memset(&wbus_ins.wbus_data, 0, sizeof(wbus_data_t));
    if (mode == WBUS_DIRECT)
        wbus_ins.wbus_data.mode = WBUS_DIRECT;
    else if (mode == WBUS_CAN)
        wbus_ins.wbus_data.mode = WBUS_CAN;
    wbus_ins.wbus_can = BSP_CAN_Init(hcan);
    if (wbus_ins.wbus_data.mode == WBUS_DIRECT)
        BSP_UART_Init(&wbus_ins.wbus_usart, &WBUS_HUART, 100000, WBUSDataProcess_UART, NULL, 0, WBUS_MAX_LEN);
    else
    {
        BSP_CAN_RegisterStdCallback(wbus_ins.wbus_can, WBUS_1_CANID, WBUSDataProcess_CAN, &wbus_ins.wbus_data);
        BSP_CAN_RegisterStdCallback(wbus_ins.wbus_can, WBUS_2_CANID, WBUSDataProcess_CAN, &wbus_ins.wbus_data);
    }
    wbus_ins.init = 1;
}

void WBUSDataProcess_UART(uint8_t *data, const uint16_t len)
{
    if (data == NULL || len != 25 || data[0] != 0x0F)
    {
        return;
    }

    wbus_data_t *wbus_data = &wbus_ins.wbus_data;

    uint16_t raw = ((uint16_t)data[1] | ((uint16_t)data[2] << 8)) & 0x07FF;
    wbus_data->ch[0] = (int16_t)raw; // 右摇杆左右
    raw = (((uint16_t)data[ 2] >> 3) | ((uint16_t)data[ 3] << 5)) & 0x07FF;
    wbus_data->ch[1] = (int16_t)raw; // 左摇杆上下
    raw = (((uint16_t)data[ 3] >> 6) | ((uint16_t)data[ 4] << 2) | ((uint16_t)data[ 5] << 10)) & 0x07FF;
    wbus_data->ch[2] = (int16_t)raw; // 右摇杆上下
    raw = (((uint16_t)data[ 5] >> 1) | ((uint16_t)data[ 6] << 7)) & 0x07FF;
    wbus_data->ch[3] = (int16_t)raw; // 左摇杆左右
    raw = (((uint16_t)data[ 6] >> 4) | ((uint16_t)data[ 7] << 4)) & 0x07FF;
    wbus_data->ch[4] = (int16_t)raw; // SA
    raw = (((uint16_t)data[ 7] >> 7) | ((uint16_t)data[ 8] << 1) | ((uint16_t)data[ 9] << 9)) & 0x07FF;
    wbus_data->ch[5] = (int16_t)raw; // SB
    raw = (((uint16_t)data[ 9] >> 2) | ((uint16_t)data[10] << 6)) & 0x07FF;
    wbus_data->ch[6] = (int16_t)raw; // SC
    raw = (((uint16_t)data[10] >> 5) | ((uint16_t)data[11] << 3)) & 0x07FF;
    wbus_data->ch[7] = (int16_t)raw; // SD
    raw = ((uint16_t)data[12] | ((uint16_t)data[13] << 8)) & 0x07FF;
    wbus_data->ch[8] = (int16_t)raw; //SE
    raw = (((uint16_t)data[13] >> 3) | ((uint16_t)data[14] << 5)) & 0x07FF;
    wbus_data->ch[9] = (int16_t)raw; // SF
    raw = (((uint16_t)data[14] >> 6) | ((uint16_t)data[15] << 2) | ((uint16_t)data[16] << 10)) & 0x07FF;
    wbus_data->ch[10] = (int16_t)raw; // SG
    raw = (((uint16_t)data[16] >> 1) | ((uint16_t)data[17] << 7)) & 0x07FF;
    wbus_data->ch[11] = (int16_t)raw; // SH
    raw = (((uint16_t)data[17] >> 4) | ((uint16_t)data[18] << 4)) & 0x07FF;
    wbus_data->ch[12] = (int16_t)raw; // LD
    raw = (((uint16_t)data[18] >> 7) | ((uint16_t)data[19] << 1) | ((uint16_t)data[20] << 9)) & 0x07FF;
    wbus_data->ch[13] = (int16_t)raw; // RD
    raw = (((uint16_t)data[20] >> 2) | ((uint16_t)data[21] << 6)) & 0x07FF;
    wbus_data->ch[14] = (int16_t)raw; // LS
    raw = (((uint16_t)data[21] >> 5) | ((uint16_t)data[22] << 3)) & 0x07FF;
    wbus_data->ch[15] = (int16_t)raw; // RS
    // wbus_data->failsafe = (data[23] & 0x08) ? 1 : 0;
    // wbus_data->frame_lost = (data[23] & 0x04) ? 1 : 0;
    for (uint16_t i = 0; i < 16; i++)
        wbus_data->ch[i] -= WBUS_CH_VALUE_OFFSET;
}

void WBUSDataProcess_CAN(const uint8_t *data, uint32_t id, void* arg)
{
    if (data == NULL || arg == NULL)
    {
        return;
    }
    wbus_data_t* wbus_data = (wbus_data_t*)arg;

    for (uint16_t i = 0; i < 4; i++)
    {
        uint16_t j = i;
        if (id >= WBUS_1_CANID && id <= WBUS_4_CANID)
            j += id - WBUS_1_CANID;
        wbus_data->ch[j] = (int16_t)(data[i * 2] | data[i * 2 + 1] << 8);
    }
}

void WBUSData_UART2CAN()
{
    if (!wbus_ins.init)
        return;
    uint8_t data[8];

    for (uint16_t i = 0; i < 4; i++)
    {
        for (uint16_t j = 0; j < 4; j++)
        {
            data[j * 2] = wbus_ins.wbus_data.ch[j + i * 4] & 0xFF;
            data[j * 2 + 1] = wbus_ins.wbus_data.ch[j + i * 4] >> 8;
        }
        BSP_CAN_Transmit(wbus_ins.wbus_can, WBUS_1_CANID+i, FDCAN_STANDARD_ID, data, 8);
    }
}

wbus_instance *Get_WBUS_Ptr(void)
{
    return &wbus_ins;
}
