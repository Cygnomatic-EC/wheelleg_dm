#ifndef DAMIAO_CHASSISL_TASK_H
#define DAMIAO_CHASSISL_TASK_H
#include "typedef.h"
#include "vmc.h"
#include "pid.h"
#include "IMU/INS/ins.h"
#include "motor/DJI/M3508/m3508.h"
#include "motor/DAMIAO/DM8009P/dm8009p.h"
#include "DBUS/dbus.h"

#define LEG_LEN_1 0.0833f // 大腿长度，单位为m
#define LEG_LEN_2 0.16f // 小腿长度，单位为m
#define MG (10.0f * 9.81f) // 质量乘以重力加速度，单位为N

#define V_COE 0.4f // 不懂？
#define PITCH_OFFSET 0.04f // 与机体中心有关？

#define RC_V_COE 0.002f
#define RC_TURN_COE 0.0005f
#define RC_ROLL_COE 0.00005f
#define RC_LEG_COE 0.00005f

#define ROLL_LIMIT 0.4f
#define LEG_LEN_MAX 0.21f
#define LEG_LEN_MIN 0.07f
#define WHEEL_TORCH_LIMIT 2.0f // 轮子扭矩限制，单位为N*m
#define F0_TORCH_LIMIT 100.0f // VMC计算得到的F0的扭矩限制
#define JOINT_TORCH_LIMIT_DEFAULT 3.0f // 关节电机扭矩限制的默认值，单位为N*m，这个值在跳跃时会被修改
#define JOINT_TORCH_LIMIT_JUMP 6.0f // 关节电机扭矩限制的跳跃值，单位为N*m，跳跃时会把这个值赋给JOINT_TORCH_LIMIT_DEFAULT来允许更大的扭矩输出
#define DEFAULT_LEG_LEN 0.10f // 默认腿长，单位为m

#define JUMP_1_LEG_LEN 0.08f // 跳跃第一阶段压缩时的腿长，单位为m
#define JUMP_1_LEG_BOUNDER 0.10f // 跳跃第一阶段认为压缩完成时腿长的边界值，单位为m
#define JUMP_2_LEG_LEN 0.40f // 跳跃第二阶段伸展时的腿长，单位为m
#define JUMP_2_LEG_BOUNDER 0.18f // 跳跃第二阶段认为伸展完成时腿长的边界值，单位为m
#define JUMP_3_LEG_LEN 0.10f // 跳跃第三阶段恢复时的腿长，单位为m
#define JUMP_3_LEG_BOUNDER 0.15f // 跳跃第三阶段认为恢复完成时腿长的边界值，单位为m

#define CHASSIS_CTRL_LOOP 1

typedef struct
{
    fp32 v_set; // 期望速度，单位为m/s
    fp32 x_set; // 期望位置，单位为m

    fp32 turn_set; // 期望yaw角，单位为rad
    fp32 roll_set; // 期望roll角，单位为rad

    fp32 phi_set; // 这个数只在右腿LQR计算用到，左腿没有，并且没人给它赋值，hyw？
    fp32 theta_set;

    fp32 leg_set; // 期望腿长，单位为m
    fp32 last_leg_set;

    fp32 v_filter; // 速度滤波后的值，单位为m/s
    fp32 x_filter; // 位置滤波后的值，单位为m

    fp32 pitchL, pitchR;
    fp32 pitchgyroL, pitchgyroR;
    fp32 roll;
    fp32 total_yaw;
    fp32 theta_err; // 两腿夹角误差

    fp32 turn_T;//yaw轴补偿
    fp32 roll_f0;//roll轴补偿

    fp32 leg_tp;//防劈叉补偿

    uint8_t start_flag;//启动标志

    uint8_t jump_flag;//右腿跳跃标志
    uint8_t jump_flag2;//左腿跳跃标志

    uint8_t prejump_flag;//预跳跃标志
    uint8_t recover_flag;//一种情况下的倒地自起标志

    uint8_t left_flag;//左腿离地标志
    uint8_t right_flag;//右腿离地标志

    fp32 left_T[2], right_T[2];
} chassis_ctrl_t;

typedef struct
{
    chassis_ctrl_t ctrl;

    vmc_leg_t *vmc_l; // 左腿VMC计算器
    vmc_leg_t *vmc_r; // 右腿VMC计算器
    pid_t leg_l_pid, leg_r_pid, tp_pid, turn_pid, roll_pid; // 腿长、防劈叉、转向、横滚的PID控制器

    INS_t *ins;
    rc_instance *rc;

    m3508_instance *wheel_motor_left;
    m3508_instance *wheel_motor_right;
    dm8009p_instance *joint_motor_left[2];
    dm8009p_instance *joint_motor_right[2];

    uint8_t init;
} chassis_t;

void chassisL_task(void *argument);
void chassisR_task(void *argument);

#endif //DAMIAO_CHASSISL_TASK_H