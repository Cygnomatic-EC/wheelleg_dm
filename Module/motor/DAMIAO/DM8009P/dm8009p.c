#include "dm8009p.h"
#include "user_lib.h"
#include <string.h>

static dm8009p_instance dm8009[DM8009P_CNT_MAX];

static void dm8009p_can_rx_callback(const uint8_t *rx_data, uint32_t id, void *arg);
DM8009P_Status_t dm8009p_init(FDCAN_HandleTypeDef *hcan, const uint8_t motor_id, const uint8_t master_id)
{
    if (hcan == NULL || motor_id == 0 || motor_id > 63)
        return DM8009P_ERROR_INVALID_PARAM;
    uint8_t index = 0;
    while (index < DM8009P_CNT_MAX)
    {
        if (dm8009[index].can_ins != NULL && dm8009[index].motor_id == motor_id)
            return DM8009P_ALREADY_INITIALIZED;
        if (dm8009[index].can_ins == NULL)
            break;
        index++;
    }
    dm8009[index].can_ins = BSP_CAN_Init(hcan);
    if (dm8009[index].can_ins == NULL)
        return DM8009P_ERROR_NOT_INIT;

    dm8009[index].motor_id = motor_id;
    dm8009[index].master_id = master_id;
    dm8009[index].mode = DM8009P_MODE_MIT;
    memset(&dm8009[index].ecd, 0, sizeof(dm8009[index].ecd));

    if (BSP_CAN_RegisterStdCallback(dm8009[index].can_ins, master_id, dm8009p_can_rx_callback, &dm8009[index].ecd) != CAN_OK)
        return DM8009P_ERROR_NOT_INIT;

    dm8009[index].init = 1;
    return DM8009P_OK;
}

static void dm8009p_can_rx_callback(const uint8_t *rx_data, uint32_t id, void *arg)
{
    if (rx_data == NULL || arg == NULL)
        return;

    dm8009p_instance *ins = (dm8009p_instance *)arg;
    dm8009p_ecd_t *fb = &ins->ecd;

    fb->err = (rx_data[0] >> 4) & 0x0F;
    fb->pos_raw = (int16_t)(rx_data[1] << 8 | rx_data[2]);
    fb->vel_raw = (int16_t)((rx_data[3] << 4 | rx_data[4] >> 4) & 0x0FFF);
    fb->tor_raw = (int16_t)((rx_data[4] & 0x0F) << 8 | rx_data[5]);
    fb->pos = dm_uint_to_float(fb->pos_raw, -DM8009P_POS_MAX, DM8009P_POS_MAX, 16);
    fb->vel = dm_uint_to_float(fb->vel_raw, -DM8009P_VEL_MAX, DM8009P_VEL_MAX, 12);
    fb->tor = dm_uint_to_float(fb->tor_raw, -DM8009P_TORQUE_MAX, DM8009P_TORQUE_MAX, 12);
    fb->mos_temp = (fp32)rx_data[6];
    fb->rotor_temp = (fp32)rx_data[7];
}

DM8009P_Status_t dm8009p_set_mode(dm8009p_instance *ins, const DM8009P_Mode_t mode)
{
    if (ins == NULL)
        return DM8009P_ERROR_INVALID_PARAM;
    ins->mode = mode;
    return DM8009P_OK;
}

DM8009P_Status_t dm8009p_enable(const dm8009p_instance *ins)
{
    if (ins == NULL || ins->can_ins == NULL)
        return DM8009P_ERROR_NOT_INIT;

    const uint8_t txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    const uint32_t mode_id = ins->mode == DM8009P_MODE_MIT ? DM8009P_MIT_BASE_ID : (ins->mode == DM8009P_MODE_POS_VEL ? DM8009P_POSVEL_BASE_ID : DM8009P_VEL_BASE_ID);
    const uint32_t tx_id = mode_id + ins->motor_id;
    if (BSP_CAN_Transmit(ins->can_ins, tx_id, FDCAN_STANDARD_ID, txdata, 8) != CAN_OK)
        return DM8009P_ERROR_CAN_TX;

    return DM8009P_OK;
}

DM8009P_Status_t dm8009p_disable(const dm8009p_instance *ins)
{
    if (ins == NULL || ins->can_ins == NULL)
        return DM8009P_ERROR_NOT_INIT;

    const uint8_t txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    const uint32_t mode_id = ins->mode == DM8009P_MODE_MIT ? DM8009P_MIT_BASE_ID : (ins->mode == DM8009P_MODE_POS_VEL ? DM8009P_POSVEL_BASE_ID : DM8009P_VEL_BASE_ID);
    const uint32_t tx_id = mode_id + ins->motor_id;
    if (BSP_CAN_Transmit(ins->can_ins, tx_id, FDCAN_STANDARD_ID, txdata, 8) != CAN_OK)
        return DM8009P_ERROR_CAN_TX;

    return DM8009P_OK;
}

DM8009P_Status_t dm8009p_ctrl_mit(const dm8009p_instance *ins,
                                  const float p_des, const float v_des,
                                  const float kp, const float kd, const float t_ff)
{
    if (ins == NULL || ins->can_ins == NULL)
        return DM8009P_ERROR_NOT_INIT;

    if (kp < 0 || kp > DM8009P_KP_MAX || kd < 0 || kd > DM8009P_KD_MAX)
        return DM8009P_ERROR_INVALID_PARAM;

    const uint16_t p_raw = dm_float_to_uint(p_des, -DM8009P_POS_MAX, DM8009P_POS_MAX, 16);
    const uint16_t v_raw = dm_float_to_uint(v_des, -DM8009P_VEL_MAX, DM8009P_VEL_MAX, 12);
    const uint16_t kp_raw = dm_float_to_uint(kp, 0.0f, DM8009P_KP_MAX, 12);
    const uint16_t kd_raw = dm_float_to_uint(kd, 0.0f, DM8009P_KD_MAX, 12);
    const uint16_t t_raw = dm_float_to_uint(t_ff, -DM8009P_TORQUE_MAX, DM8009P_TORQUE_MAX, 12);

    uint8_t txdata[8];
    txdata[0] = (uint8_t)(p_raw >> 8);
    txdata[1] = (uint8_t)(p_raw & 0xFF);
    txdata[2] = (uint8_t)((v_raw >> 4) & 0xFF);
    txdata[3] = (uint8_t)(((v_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F));
    txdata[4] = (uint8_t)(kp_raw & 0xFF);
    txdata[5] = (uint8_t)((kd_raw >> 4) & 0xFF);
    txdata[6] = (uint8_t)(((kd_raw & 0x0F) << 4) | ((t_raw >> 8) & 0x0F));
    txdata[7] = t_raw & 0xFF;

    const uint32_t tx_id = DM8009P_MIT_BASE_ID + ins->motor_id;
    if (BSP_CAN_Transmit(ins->can_ins, tx_id, FDCAN_STANDARD_ID, txdata, 8) != CAN_OK)
        return DM8009P_ERROR_CAN_TX;

    return DM8009P_OK;
}

/* 位置速度模式控制 */
DM8009P_Status_t dm8009p_ctrl_pos_vel(const dm8009p_instance *ins, const float p_des, const float v_des)
{
    if (ins == NULL || ins->can_ins == NULL)
        return DM8009P_ERROR_NOT_INIT;

    uint8_t txdata[8];

    memcpy(txdata, &p_des, 4);
    memcpy(txdata + 4, &v_des, 4);

    uint32_t tx_id = DM8009P_POSVEL_BASE_ID + ins->motor_id;
    if (BSP_CAN_Transmit(ins->can_ins, tx_id, FDCAN_STANDARD_ID, txdata, 8) != CAN_OK)
        return DM8009P_ERROR_CAN_TX;

    return DM8009P_OK;
}

DM8009P_Status_t dm8009p_ctrl_vel(const dm8009p_instance *ins, const float v_des)
{
    if (ins == NULL || ins->can_ins == NULL)
        return DM8009P_ERROR_NOT_INIT;

    uint8_t txdata[8];
    
    memcpy(txdata, &v_des, 4);
    memset(txdata + 4, 0, 4);

    const uint32_t tx_id = DM8009P_VEL_BASE_ID + ins->motor_id;
    if (BSP_CAN_Transmit(ins->can_ins, tx_id, FDCAN_STANDARD_ID, txdata, 8) != CAN_OK)
        return DM8009P_ERROR_CAN_TX;

    return DM8009P_OK;
}

dm8009p_instance *Get_DM8009P_Ptr(const uint8_t motor_id)
{
    if (motor_id == 0 || motor_id > 63)
        return NULL;
    for (uint8_t i = 0; i < DM8009P_CNT_MAX; i++)
    {
        if (dm8009[i].can_ins != NULL && dm8009[i].motor_id == motor_id)
            return &dm8009[i];
    }
    return NULL;
}