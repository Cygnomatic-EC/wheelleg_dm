#include "ins.h"
#include "../../../BSP/algorithm/pid_enhanced.h"
#include "IMU/EKF/QuaternionEKF.h"
#include "pwm/bsp_pwm.h"
#include "dwt/bsp_dwt.h"
#include "robot_config.h"

#define X 0
#define Y 1
#define Z 2

static PID_plus_t TempCtrl = {0};
static float RefTemp = 40;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

void INS_Task(void *argument);
osThreadId_t INStaskHandle;
const osThreadAttr_t insTask_attr = {
    .name = "insTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityHigh,
  };
static INS_t INS;

void INS_Init()
{
    IMU_Data_t *bmi088 = Get_BMI088_Ptr();
    if (bmi088 == NULL || !bmi088->init)
        return ;
    INS.imu = bmi088;
    BSP_PWM_Init(&INS.pwm, &htim3, TIM_CHANNEL_4);

    IMU_QuaternionEKF_Init(10, 0.001f, 10000000, 1, 0);
    PID_plus_Init(&TempCtrl, 2000, 300, 0, 1000, 20, 0, 0, 0, 0, 0, 0, 0);
    INS.ins.AccelLPF = 0.0085f;
    INStaskHandle = osThreadNew(INS_Task, NULL, &insTask_attr);

}

void INS_Update()
{
    if (!INS.imu->init)
        return;
    /* === IMU update === */
    BMI088_Read();

    static uint32_t INS_DWT_Count = 0;
    static float dt = 0, t = 0;

    const float gravity[3] = {0, 0, 9.8015f};
    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    INS.ins.Accel[X] = INS.imu->Accel[X];
    INS.ins.Accel[Y] = INS.imu->Accel[Y];
    INS.ins.Accel[Z] = INS.imu->Accel[Z];
    INS.ins.Gyro[X]  = INS.imu->Gyro[X];
    INS.ins.Gyro[Y]  = INS.imu->Gyro[Y];
    INS.ins.Gyro[Z]  = INS.imu->Gyro[Z];

    // 核心函数,EKF更新四元数
    IMU_QuaternionEKF_Update(
        INS.ins.Gyro[X], INS.ins.Gyro[Y], INS.ins.Gyro[Z],
        INS.ins.Accel[X], INS.ins.Accel[Y], INS.ins.Accel[Z],
        dt
    );

    const QEKF_INS_t *QEKF_INS = Get_QEKF_INS();
    memcpy(INS.ins.q, QEKF_INS->q, sizeof(QEKF_INS->q));

    // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
    BodyFrameToEarthFrame(xb, INS.ins.xn, INS.ins.q);
    BodyFrameToEarthFrame(yb, INS.ins.yn, INS.ins.q);
    BodyFrameToEarthFrame(zb, INS.ins.zn, INS.ins.q);

    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, INS.ins.q);

    for (uint8_t i = 0; i < 3; i++)
    {
        INS.ins.MotionAccel_b[i] =
            (INS.ins.Accel[i] - gravity_b[i]) * dt / (INS.ins.AccelLPF + dt) +
            INS.ins.MotionAccel_b[i] * INS.ins.AccelLPF / (INS.ins.AccelLPF + dt);
    }

    BodyFrameToEarthFrame(INS.ins.MotionAccel_b, INS.ins.MotionAccel_n, INS.ins.q);

    // 获取最终数据
    INS.ins.Yaw            = QEKF_INS->Yaw / 360 * 2 * PI;
    INS.ins.Pitch          = QEKF_INS->Pitch / 360 * 2 * PI;
    INS.ins.Roll           = QEKF_INS->Roll / 360 * 2 * PI;
    INS.ins.YawTotalAngle  = QEKF_INS->YawTotalAngle / 360 * 2 * PI;

    /* === temperature control (500Hz) === */
    static uint32_t sys_time_ms = 0;
    static uint32_t last_temp_ctrl_ms = 0;
    sys_time_ms = (uint32_t)DWT_GetTimeline_ms();
    if (sys_time_ms - last_temp_ctrl_ms >= 2)
    {
        last_temp_ctrl_ms = sys_time_ms;
        IMU_Temperature_Ctrl();
    }
}

void INS_Task(void *argument)
{
    while(1)
    {
        osDelay(1);
        INS_Update();
    }
}

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, const float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, const float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

void IMU_Temperature_Ctrl()
{
    PID_plus_Calculate(&TempCtrl, INS.imu->Temperature, RefTemp);
    fp32 temp_out = TempCtrl.Output;
    float_constrain(&temp_out, 0, (fp32)UINT32_MAX * 1.0f);
    BSP_PWM_SetValue(&INS.pwm, (uint16_t)temp_out);
}

INS_t *Get_INS_Ptr()
{
    return &INS;
}