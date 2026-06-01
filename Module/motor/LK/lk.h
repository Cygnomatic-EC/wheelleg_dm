#ifndef STANDARD_ROBOT_C_LK_H
#define STANDARD_ROBOT_C_LK_H
#include "can/bsp_can.h"
#include "typedef.h"
#include "arm_math.h"

#define LK_CNT_MAX 5

#define LK_TX_MIN 0x141
#define LK_TX_MAX 0x160

#define LK_ECD_MAX 65535

#define LK_MAX_IQ 2048

#define LK_IQ_RESOLUTION (33.0f / 4096.0f)
#define LK_SPEED_RESOLUTION 1.0f
#define LK_ECD_RESOLUTION (2.0f * PI / 65536.0f)
#define LK_TORQUE_CONSTANT 0.81f // 0.32f(16匝)

typedef enum
{
    LK_OK = 0,
    LK_ERROR_INVALID_PARAM = 1,
    LK_ALREADY_INITIALIZED = 2,
    LK_ERROR = 3
} LK_Status_t;

typedef enum
{
    CMD_LK_INCREMENT_ANGLE_CONTROL = 0xA8,
    CMD_LK_TORQUE_CONTROL = 0xA1,
    CMD_LK_ANGLE_CONTROL = 0xA6,
    CMD_LK_SPEED_CONTROL = 0xA2,
    CMD_LK_READ_MEASURE  = 0x9C,
    CMD_LK_READ_ENCODER = 0x90,
    CMD_LK_READ_CONTROL_PARAM = 0xC0,
    CMD_LK_WRITE_CONTROL_PARAM = 0xC1
} LK_CMD_t;

typedef enum
{
    PARAM_LK_ANGLE_PID = 0x0A,
    PARAM_LK_SPEED_PID = 0x0B,
    PARAM_LK_IQ_PID = 0x0C
} LK_PARAM_t;

typedef struct
{
    uint16_t ecd_raw;
    fp32 angle;
    int16_t speed;
    int16_t iq_raw;
    fp32 torque;
    int8_t temperature;
    int16_t last_ecd;

    fp32 angle_offset;
}lk_ecd_t;

typedef struct
{
    CAN_Instance_t *can_ins;
    uint32_t txid;
    lk_ecd_t ecd;
    uint8_t init;
}lk_instance;

LK_Status_t lk_init(FDCAN_HandleTypeDef *hcan, uint32_t txid);
void lk_speed_init(const lk_instance* lk_ins, const uint16_t pid_v[3]);
LK_Status_t lk_ctrl_torque(const lk_instance* lk_ins, fp32 torqueControl);
LK_Status_t lk_ctrl_speed(const lk_instance* lk_ins, uint16_t iqControl, uint32_t speedControl);
LK_Status_t lk_set_pid(const lk_instance* lk_ins, uint8_t param, uint16_t kp, uint16_t ki, uint16_t kd);
LK_Status_t lk_get_measure(const lk_instance* lk_ins);
lk_instance *Get_LK_Ptr(uint32_t txid);

#endif //STANDARD_ROBOT_C_LK_H