#include "CBoard_gimbal.h"
#include "user_lib.h"

static cboard_gimbal_t cboard_gimbal;

void CBoard_RefereeCallback(const uint8_t* data, uint32_t id, void* arg);
void CBoard_Gimbal_Init(FDCAN_HandleTypeDef* hcan)
{
    if (hcan == NULL)
        return;
    memset(&cboard_gimbal, 0, sizeof(cboard_gimbal_t));
    cboard_gimbal.can_instance = BSP_CAN_Init(hcan);
    BSP_CAN_RegisterStdCallback(cboard_gimbal.can_instance, CMD_ID_REFEREE_GET, CBoard_RefereeCallback, &cboard_gimbal);
    cboard_gimbal.init = 1;
}

void CBoard_RefereeCallback(const uint8_t* data, uint32_t id, void* arg)
{
    if (data == NULL || arg == NULL)
        return;
    cboard_gimbal_t* cbord_gimbal_ptr = (cboard_gimbal_t*)arg;
    cbord_gimbal_ptr->game_start = data[0] & 0x01;
    cbord_gimbal_ptr->camp = (data[0] >> 1) & 0x01;
    cbord_gimbal_ptr->attitude = (data[0] >> 2) & 0x01;
    cbord_gimbal_ptr->shoot_heat = (uint16_t)(data[2] << 8 | data[1]);
    cbord_gimbal_ptr->bullet_allow = (uint16_t)(data[4] << 8 | data[3]);
}

void CBoard_IMU_Transmit(const fp32 imu_yaw)
{
    cboard_gimbal_t* cbord_gimbal_ptr = &cboard_gimbal;
    if (cbord_gimbal_ptr == NULL || !cbord_gimbal_ptr->init)
        return;
    uint8_t data[8];

    pack_float_to_4bytes(imu_yaw, &data[0]);
    memset(data + 4, 0, 4);

    BSP_CAN_Transmit(cbord_gimbal_ptr->can_instance, CMD_ID_IMU_TRAN, FDCAN_STANDARD_ID, data, 8);
}

cboard_gimbal_t *Get_CBoard_Gimbal_Ptr(void)
{
    return &cboard_gimbal;
}