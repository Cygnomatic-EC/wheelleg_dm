#include "gm6020.h"
#include "user_lib.h"

static gm6020_instance gm6020[2];

void get_gm6020_measure(const uint8_t* rx_data, uint32_t id, void* arg);
GM6020_Status_t gm6020_init(FDCAN_HandleTypeDef *hcan, const uint32_t txid, const uint8_t num)
{
    if (txid == GM6020_TX_1 || txid == GM6020_TX_2)
    {
        const uint8_t index = (txid == GM6020_TX_1) ? 0 : 1;
        gm6020[index].can_ins = BSP_CAN_Init(hcan);
        gm6020[index].num = num;
        gm6020[index].txid = txid;

        if (txid == GM6020_TX_1)
        {
            if (num > GM6020_MAXNUM_1)
                return GM6020_ERROR_INVALID_PARAM;
            for (uint8_t i = 0; i < num; i++){
                BSP_CAN_RegisterStdCallback(gm6020[index].can_ins, GM6020_RX_1+i, get_gm6020_measure, &gm6020[index].ecd[i]);
            }
        }
        else
        {
            if (num > GM6020_MAXNUM_2)
                return GM6020_ERROR_INVALID_PARAM;
            for (uint8_t i = 0; i < num; i++){
                BSP_CAN_RegisterStdCallback(gm6020[index].can_ins, GM6020_RX_2+i, get_gm6020_measure, &gm6020[index].ecd[i]);
            }
        }
        gm6020[index].init = 1;
        return GM6020_OK;
    }
    return GM6020_ERROR_INVALID_PARAM;
}

GM6020_Status_t gm6020_ctrl_voltage(const gm6020_instance* gm6020_ins, int16_t voltage[4])
{
    if (gm6020_ins == NULL || !gm6020_ins->init)
        return GM6020_ERROR;

    const CAN_Instance_t* can_ins = gm6020_ins->can_ins;
    uint8_t txdata[8];
    for (uint8_t i = 0; i < 4; i++)
    {
        int16_constrain(&voltage[i], -GM6020_VOLTAGGE_MAX, GM6020_VOLTAGGE_MAX);
        txdata[i * 2] = voltage[i] >> 8;
        txdata[i * 2 + 1] = voltage[i];
    }
    if (BSP_CAN_Transmit(can_ins, gm6020_ins->txid, FDCAN_STANDARD_ID, txdata, 8) != CAN_OK)
        return GM6020_ERROR;

    return GM6020_OK;
}

GM6020_Status_t gm6020_ctrl_current(const gm6020_instance* gm6020_ins, int16_t current[4])
{
    if (gm6020_ins == NULL || !gm6020_ins->init)
        return GM6020_ERROR;

    const CAN_Instance_t* can_ins = gm6020_ins->can_ins;
    uint8_t txdata[8];
    for (uint8_t i = 0; i < 4; i++)
    {
        int16_constrain(&current[i], -GM6020_CURRENT_MAX, GM6020_CURRENT_MAX);
        txdata[i * 2] = current[i] >> 8;
        txdata[i * 2 + 1] = current[i] >> 8;
    }
    if (BSP_CAN_Transmit(can_ins, gm6020_ins->txid - 1, FDCAN_STANDARD_ID, txdata, 8) != CAN_OK)
        return GM6020_ERROR;

    return GM6020_OK;
}

void get_gm6020_measure(const uint8_t* rx_data, uint32_t id, void* arg)
{
    if(rx_data == NULL || arg == NULL)
        return;

    gm6020_ecd_t *ecd = (gm6020_ecd_t*)arg;
    ecd->last_ecd = (int16_t)ecd->ecd;
    ecd->ecd = (uint16_t)((rx_data)[0] << 8 | (rx_data)[1]);
    ecd->speed = (int16_t)((rx_data)[2] << 8 | (rx_data)[3]);
    ecd->current = (int16_t)((rx_data)[4] << 8 | (rx_data)[5]);
    ecd->temperature = (rx_data)[6];
}

gm6020_instance *Get_GM6020_Ptr(const uint16_t txid)
{
    if (txid == GM6020_TX_1)
        return &gm6020[0];
    if (txid == GM6020_TX_2)
        return &gm6020[1];

    return NULL;
}