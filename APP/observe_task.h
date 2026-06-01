#ifndef DAMIAO_OBSERVE_TASK_H
#define DAMIAO_OBSERVE_TASK_H
#include "typedef.h"
#include "kalman_filter.h"
#include "motor/DJI/M3508/m3508.h"
#include "motor/LK/lk.h"
#include "IMU/INS/ins.h"
#include "vmc.h"

#define WHEEL_RADIUS 0.05f
#define WHEEL_REDUCTION_RATIO 19.0f
#define WHEEL_RPM2RAD (3.1415926f / (60.0f * WHEEL_REDUCTION_RATIO))
#define WHEEL_DPS2RAD (3.1415926f / 180.0f)
#define OBESERVE_TASK_LOOP 1


typedef struct
{
    KalmanFilter_t kf;
    fp32 fliter_output[2];
    vmc_leg_t *leg_left;
    vmc_leg_t *leg_right;
#ifdef INFANTRY
    m3508_instance *wheel_motor_left;
    m3508_instance *wheel_motor_right;
#elif HERO
    lk_instance *wheel_motor_left;
    lk_instance *wheel_motor_right;
#endif
    INS_t *ins;

    fp32 wheel_rad;
} Observe_t;

void Observe_Task(void *argument);
fp32 Get_Observe_Velocity(void);
fp32 Get_Observe_Position_Delta(void);

#endif //DAMIAO_OBSERVE_TASK_H