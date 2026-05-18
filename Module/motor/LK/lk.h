#ifndef STANDARD_ROBOT_C_LK_H
#define STANDARD_ROBOT_C_LK_H
#include "can/bsp_can.h"
#include "typedef.h"

#define LK_CNT_MAX 5

#define LK_TX_MIN 0x141
#define LK_TX_MAX 0x160

#define LK_ECD_MAX 65535
#define LK_ECD_IN_ZERO 0x4520

#define LK_MAX_IQ 2048

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
    uint16_t ecd;
    int16_t speed; // 1dps/LSB
    int16_t iq;
    int8_t temperature;
    int16_t last_ecd;

    uint16_t ecd_offset; // 记录初始编码器值用于后续计算相对初始位置角度
    uint16_t zero_offset; // 记录初始编码器值与绝对零位的差值
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
LK_Status_t lk_ctrl_speed(const lk_instance* lk_ins, uint16_t iqControl, uint32_t speedControl);
LK_Status_t lk_set_pid(const lk_instance* lk_ins, uint8_t param, uint16_t kp, uint16_t ki, uint16_t kd);
LK_Status_t lk_get_measure(const lk_instance* lk_ins);
lk_instance *Get_LK_Ptr(uint32_t txid);

#endif //STANDARD_ROBOT_C_LK_H