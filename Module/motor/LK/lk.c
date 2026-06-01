#include "lk.h"
#include "user_lib.h"

static lk_instance lk[LK_CNT_MAX];

void lk_callback(const uint8_t* rx_data, uint32_t id, void* arg);
LK_Status_t lk_init(FDCAN_HandleTypeDef *hcan, const uint32_t txid)
{
    uint8_t index = 0;
    if (txid < LK_TX_MIN || txid > LK_TX_MAX)
        return LK_ERROR_INVALID_PARAM;
    while (index < LK_CNT_MAX)
    {
        if (lk[index].init && lk[index].txid == txid)
            return LK_ALREADY_INITIALIZED;
        if (!lk[index].init)
            break;
        index++;
    }
    lk[index].can_ins = BSP_CAN_Init(hcan);
    lk[index].txid = txid;
    BSP_CAN_RegisterStdCallback(lk[index].can_ins, txid, lk_callback, &lk[index].ecd);
    lk[index].init = 1;
    return LK_OK;
}

void lk_speed_init(const lk_instance* lk_ins, const uint16_t pid_v[3])
{
    if (lk_ins == NULL) return;
    while (lk_ins->ecd.angle_offset == 0)
    {
        lk_get_measure(lk_ins);
        osDelay(5);
    }
    osDelay(1000);
    for (int i = 0; i < 10; i++)
    {
        lk_set_pid(lk_ins, PARAM_LK_SPEED_PID, pid_v[0], pid_v[1], pid_v[2]);
        osDelay(5);
    }
}

LK_Status_t lk_ctrl_speed(const lk_instance* lk_ins, uint16_t iqControl, uint32_t speedControl)
{
    if (lk_ins == NULL)
        return LK_ERROR;

    const CAN_Instance_t* can_ins = lk_ins->can_ins;
    uint8_t txdata[8];

    txdata[0] = CMD_LK_SPEED_CONTROL;
    txdata[1] = 0x00;
    txdata[2] = ((uint8_t *)&iqControl)[0];
    txdata[3] = ((uint8_t *)&iqControl)[1];
    txdata[4] = ((uint8_t *)&speedControl)[0];
    txdata[5] = ((uint8_t *)&speedControl)[1];
    txdata[6] = ((uint8_t *)&speedControl)[2];
    txdata[7] = ((uint8_t *)&speedControl)[3];

    if (BSP_CAN_Transmit(can_ins, lk_ins->txid, FDCAN_STANDARD_ID, txdata, 8) != CAN_OK)
        return LK_ERROR;

    return LK_OK;
}

LK_Status_t lk_ctrl_torque(const lk_instance* lk_ins, const fp32 torqueControl)
{
    if (lk_ins == NULL)
        return LK_ERROR;

    const CAN_Instance_t* can_ins = lk_ins->can_ins;

    uint16_t iqControl = (uint16_t)(torqueControl / LK_IQ_RESOLUTION / LK_TORQUE_CONSTANT);
    uint8_t txdata[8];

    txdata[0] = CMD_LK_TORQUE_CONTROL;
    txdata[1] = 0x00;
    txdata[2] = 0x00;
    txdata[3] = 0x00;
    txdata[4] = ((uint8_t *)&iqControl)[0];
    txdata[5] = ((uint8_t *)&iqControl)[1];
    txdata[6] = 0x00;
    txdata[7] = 0x00;

    if (BSP_CAN_Transmit(can_ins, lk_ins->txid, FDCAN_STANDARD_ID, txdata, 8) != CAN_OK)
        return LK_ERROR;

    return LK_OK;
}

// 仅对MS电机可以开环控制
LK_Status_t lk_ctrl_open(const lk_instance* lk_ins, uint16_t iqControl, uint32_t speedControl)
{
    if (lk_ins == NULL)
        return LK_ERROR;

    const CAN_Instance_t* can_ins = lk_ins->can_ins;
    uint8_t txdata[8];

    txdata[0] = CMD_LK_SPEED_CONTROL;
    txdata[1] = 0x00;
    txdata[2] = ((uint8_t *)&iqControl)[0];
    txdata[3] = ((uint8_t *)&iqControl)[1];
    txdata[4] = ((uint8_t *)&speedControl)[0];
    txdata[5] = ((uint8_t *)&speedControl)[1];
    txdata[6] = ((uint8_t *)&speedControl)[2];
    txdata[7] = ((uint8_t *)&speedControl)[3];

    if (BSP_CAN_Transmit(can_ins, lk_ins->txid, FDCAN_STANDARD_ID, txdata, 8) != CAN_OK)
        return LK_ERROR;

    return LK_OK;
}

// param: CONTROL_PARAM_LK_ANGLE_PID CONTROL_PARAM_LK_SPEED_PID CONTROL_PARAM_LK_IQ_PID
LK_Status_t lk_set_pid(const lk_instance* lk_ins, const uint8_t param, uint16_t kp, uint16_t ki, uint16_t kd)
{
    if (lk_ins == NULL)
        return LK_ERROR;

    const CAN_Instance_t* can_ins = lk_ins->can_ins;
    uint8_t txdata[8];

    txdata[0] = CMD_LK_WRITE_CONTROL_PARAM;
    txdata[1] = param;
    txdata[2] = ((uint8_t *)&kp)[0];
    txdata[3] = ((uint8_t *)&kp)[1];
    txdata[4] = * (uint8_t*)(&ki);
    txdata[5] = *((uint8_t*)(&ki)+1);
    txdata[6] = *((uint8_t*)(&kd)+0);
    txdata[7] = *((uint8_t*)(&kd)+1);

    if (BSP_CAN_Transmit(can_ins, lk_ins->txid, FDCAN_STANDARD_ID, txdata, 8) != CAN_OK)
        return LK_ERROR;

    return LK_OK;
}

LK_Status_t lk_get_measure(const lk_instance* lk_ins)
{
    if (lk_ins == NULL)
        return LK_ERROR;

    const CAN_Instance_t* can_ins = lk_ins->can_ins;
    uint8_t txdata[8];

    txdata[0] = CMD_LK_READ_MEASURE;
    txdata[1] = 0x00;
    txdata[2] = 0x00;
    txdata[3] = 0x00;
    txdata[4] = 0x00;
    txdata[5] = 0x00;
    txdata[6] = 0x00;
    txdata[7] = 0x00;

    if (BSP_CAN_Transmit(can_ins, lk_ins->txid, FDCAN_STANDARD_ID, txdata, 8) != CAN_OK)
        return LK_ERROR;

    return LK_OK;
}

void get_lk_ecd(const uint8_t* rx_data, void* arg);
void lk_callback(const uint8_t* rx_data, uint32_t id, void* arg)
{
    if(rx_data == NULL || arg == NULL)
        return;

    switch(rx_data[0])
    {
        case CMD_LK_READ_MEASURE:
        case CMD_LK_TORQUE_CONTROL:
        case CMD_LK_SPEED_CONTROL:
        case CMD_LK_ANGLE_CONTROL:
        case CMD_LK_INCREMENT_ANGLE_CONTROL:
            get_lk_ecd(&rx_data[0], arg);
            break;
        case CMD_LK_READ_ENCODER:
            // get_motor_LK_ecd_data(motor_LK->motor_LK_ecd_data, rx_data);
            // 大概率用不到，不想写了
        case CMD_LK_READ_CONTROL_PARAM:
            // get_motor_LK_control_param(motor_LK->motor_LK_pid, rx_data);
            // 可以用上位机读取
        default:
            break;
    }
}

void get_lk_ecd(const uint8_t* rx_data, void* arg)
{
    if(rx_data == NULL || arg == NULL)
        return;

    lk_ecd_t *ecd = (lk_ecd_t*)arg;
    ecd->temperature = (int8_t)   (rx_data)[1];
    ecd->iq_raw        = (int16_t)  ((rx_data)[3]<<8 | (rx_data)[2]);
    ecd->speed     = (int16_t)  ((rx_data)[5]<<8 | (rx_data)[4]);
    ecd->ecd_raw       = (uint16_t) ((rx_data)[7]<<8 | (rx_data)[6]);
    ecd->angle         = (fp32)ecd->ecd_raw * LK_ECD_RESOLUTION;
    ecd->torque        = (fp32)ecd->iq_raw * LK_IQ_RESOLUTION * LK_ECD_RESOLUTION;
    if (ecd->angle_offset == 0.0f)
    {
        ecd->angle_offset = ecd->angle;
    }
}

lk_instance *Get_LK_Ptr(const uint32_t txid)
{
    for (uint8_t i = 0; i < LK_CNT_MAX; i++)
    {
        if (lk[i].init && lk[i].txid == txid)
            return &lk[i];
    }
    return NULL;
}