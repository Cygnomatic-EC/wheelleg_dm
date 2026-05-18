#ifndef DM4310_H
#define DM4310_H

#include "typedef.h"
#include "can/bsp_can.h"

#define DM4310_CNT_MAX         5

#define DM4310_MIT_BASE_ID     0x000
#define DM4310_POSVEL_BASE_ID  0x100
#define DM4310_VEL_BASE_ID     0x200

#define DM4310_KP_MAX          500.0f
#define DM4310_KD_MAX          5.0f

#define DM4310_POS_MAX         12.5f //还没改
#define DM4310_VEL_MAX         45.0f //还没改
#define DM4310_TORQUE_MAX      54.0f //还没改

typedef enum {
    DM4310_MODE_MIT = 0,
    DM4310_MODE_POS_VEL,
    DM4310_MODE_VEL
} DM4310_Mode_t;

typedef enum {
    DM4310_OK = 0,
    DM4310_ERROR_INVALID_PARAM = 1,
    DM4310_ERROR_NOT_INIT = 2,
    DM4310_ERROR_CAN_TX = 3,
    DM4310_ERROR_INVALID_MODE = 4,
    DM4310_ALREADY_INITIALIZED = 5
} DM4310_Status_t;

typedef struct {
    uint8_t id;
    int16_t pos_raw;
    int16_t  vel_raw;
    int16_t  tor_raw;
    fp32 pos;
    fp32 vel;
    fp32 tor;
    fp32  mos_temp;
    fp32  rotor_temp;
    uint8_t  err;       
} dm4310_ecd_t;

typedef struct {
    CAN_Instance_t     *can_ins;    
    uint8_t            motor_id;
    uint8_t            master_id;
    DM4310_Mode_t     mode;
    dm4310_ecd_t      ecd;
    uint8_t            init;
} dm4310_instance;

DM4310_Status_t dm4310_init(FDCAN_HandleTypeDef *hcan, uint8_t motor_id, uint8_t master_id);

DM4310_Status_t dm4310_set_mode(dm4310_instance *ins, DM4310_Mode_t mode);

DM4310_Status_t dm4310_enable(const dm4310_instance *ins);

DM4310_Status_t dm4310_disable(const dm4310_instance *ins);

DM4310_Status_t dm4310_ctrl_mit(const dm4310_instance *ins,
                                  float p_des, float v_des,
                                  float kp, float kd, float t_ff);

DM4310_Status_t dm4310_ctrl_pos_vel(const dm4310_instance *ins,
                                      float p_des, float v_des);

DM4310_Status_t dm4310_ctrl_vel(const dm4310_instance *ins, float v_des);
dm4310_instance *Get_DM4310_Ptr(uint8_t motor_id);

#endif /* DM4310_H */