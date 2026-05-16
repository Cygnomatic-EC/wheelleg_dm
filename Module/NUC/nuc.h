#ifndef STANDARD_ROBOT_C_NUC_H
#define STANDARD_ROBOT_C_NUC_H
#include "usart/bsp_uart.h"
#include "typedef.h"

typedef enum
{
    CMD_ID_SELF_DECISION = 0x401,
    CMD_ID_CHASSIS_CTRL = 0x402,
    CMD_ID_BIG_GIMBAL_CTRL = 0x403,
} nuc_cmd_id_t;

typedef struct
{
    UART_Instance_t nuc_uart;
    fp32 vx, vy, vw;
    fp32 gimbal_yaw;
    uint8_t gimbal_scan, last_gimbal_scan;
    uint8_t chassis_spin;
    uint8_t use_cap;
    uint32_t self_decision;
    uint8_t init;
} nuc_ctrl_t;

void NUC_Init(UART_HandleTypeDef *nuc_uart);
nuc_ctrl_t *Get_NUC_Ctrl_Instance(void);
void NUC_Referee_Tran(const uint8_t* data, uint16_t len);

#endif //STANDARD_ROBOT_C_NUC_H