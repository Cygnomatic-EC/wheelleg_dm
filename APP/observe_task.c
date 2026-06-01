#include "observe_task.h"
#include "cmsis_os2.h"
const float vaEstimateKF_F[4] = {1.0f, 0.003f,
                           0.0f, 1.0f};

const float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};

float vaEstimateKF_Q[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};

float vaEstimateKF_R[4] = {200.0f, 0.0f,
                            0.0f,  200.0f};

float vaEstimateKF_K[4];

const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};
static Observe_t obs;

void Observe_Init(const float wheel_rad)
{
    obs.leg_left = Get_VMC_Leg(LEFT);
    obs.leg_right = Get_VMC_Leg(RIGHT);
#ifdef INFANTRY
    obs.wheel_motor_left = Get_M3508_Ptr(M3508_TX_1);
    obs.wheel_motor_right = Get_M3508_Ptr(M3508_TX_2);
#elif HERO
    obs.wheel_motor_left = Get_LK_Ptr(0x142);
    obs.wheel_motor_right = Get_LK_Ptr(0x141);
#endif
    obs.ins = Get_INS_Ptr();
    obs.wheel_rad = wheel_rad;

    Kalman_Filter_Init(&obs.kf, 2, 0, 2);
    memcpy(obs.kf.F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(obs.kf.P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(obs.kf.Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(obs.kf.R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(obs.kf.H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

void Observe_Update(const float acc, const float vel)
{
    obs.kf.MeasuredVector[0] =	vel;
    obs.kf.MeasuredVector[1] = acc;

    Kalman_Filter_Update(&obs.kf);

    for (uint8_t i = 0; i < 2; i++)
    {
        obs.fliter_output[i] = obs.kf.FilteredValue[i];
    }
}

void Observe_Task(void *argument)
{
    Observe_Init(WHEEL_RADIUS);
    static float wr, wl=0.0f;
    static float vrb, vlb=0.0f;
    static float aver_v=0.0f;
    while (1)
    {
#ifdef INFANTRY
        wr= -(fp32)obs.wheel_motor_right->ecd->speed * WHEEL_RPM2RAD- obs.ins->ins.Gyro[1] + obs.leg_right->d_alpha;//右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
        vrb=wr * obs.wheel_rad + obs.leg_right->L0 * obs.leg_right->d_theta*arm_cos_f32(obs.leg_right->theta) + obs.leg_right->d_L0 * arm_sin_f32(obs.leg_right->theta);//机体b系的速度

        wl= -(fp32)obs.wheel_motor_left->ecd->speed * WHEEL_RPM2RAD - obs.ins->ins.Gyro[1] + obs.leg_left->d_alpha;//左边驱动轮转子相对大地角速度，这里定义的是顺时针为正
        vlb=wl * obs.wheel_rad + obs.leg_left->L0 * obs.leg_left->d_theta*arm_cos_f32(obs.leg_left->theta) + obs.leg_left->d_L0 * arm_sin_f32(obs.leg_left->theta);//机体b系的速度
#elif HERO
        wr= -(fp32)obs.wheel_motor_right->ecd.speed * WHEEL_DPS2RAD- obs.ins->ins.Gyro[1] + obs.leg_right->d_alpha;//右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
        vrb=wr * obs.wheel_rad + obs.leg_right->L0 * obs.leg_right->d_theta*arm_cos_f32(obs.leg_right->theta) + obs.leg_right->d_L0 * arm_sin_f32(obs.leg_right->theta);//机体b系的速度

        wl= -(fp32)obs.wheel_motor_left->ecd.speed * WHEEL_DPS2RAD - obs.ins->ins.Gyro[1] + obs.leg_left->d_alpha;//左边驱动轮转子相对大地角速度，这里定义的是顺时针为正
        vlb=wl * obs.wheel_rad + obs.leg_left->L0 * obs.leg_left->d_theta*arm_cos_f32(obs.leg_left->theta) + obs.leg_left->d_L0 * arm_sin_f32(obs.leg_left->theta);//机体b系的速度
#endif
        aver_v=(vrb - vlb) / 2.0f;
        Observe_Update(-obs.ins->ins.MotionAccel_b[0],aver_v);
        osDelay(OBESERVE_TASK_LOOP);
    }
}

fp32 Get_Observe_Velocity(void)
{
    return obs.fliter_output[0];
}

fp32 Get_Observe_Position_Delta(void)
{
    return obs.fliter_output[0] * OBESERVE_TASK_LOOP / 1000.0f;
}