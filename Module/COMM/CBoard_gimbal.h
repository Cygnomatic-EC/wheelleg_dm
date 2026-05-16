#ifndef STANDARD_ROBOT_C_CBOARD_GIMBAL_H
#define STANDARD_ROBOT_C_CBOARD_GIMBAL_H
#include "typedef.h"
#include "can/bsp_can.h"

typedef enum
{
    CMD_ID_REFEREE_GET = 0x601,
    CMD_ID_RC_TRAN = 0x600,
    CMD_ID_IMU_TRAN = 0x603,
} cboard_gimbal_cmd_id_t;

typedef struct
{
    CAN_Instance_t* can_instance;
    uint8_t game_start;
    uint8_t camp;
    uint8_t attitude;
    uint16_t shoot_heat;
    uint16_t bullet_allow;
    uint8_t init;
} cboard_gimbal_t;

void CBoard_Gimbal_Init(FDCAN_HandleTypeDef* hcan);
void CBoard_RC_Transmit(const int16_t ch[4], const uint8_t sw[2], int16_t roll);
void CBoard_IMU_Transmit(fp32 imu_yaw);
cboard_gimbal_t *Get_CBoard_Gimbal_Ptr(void);
#endif //STANDARD_ROBOT_C_CBOARD_GIMBAL_H