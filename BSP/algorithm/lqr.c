#include "lqr.h"

float LQR_K_calc(const float *coe, const float len)
{
    return coe[0] * len * len * len + coe[1] * len * len + coe[2] * len + coe[3];
}

void LQR_Calc(fp32 *T, const fp32 *K, const fp32 *err)
{
    T[0] = 0.0f; T[1] = 0.0f;
    for (uint8_t i = 0; i < 6; i++)
    {
        T[0] += K[i] * err[i];
        T[1] += K[i + 6] * err[i];
    }
}

void LQR_Calc_air(fp32 *T, const fp32 *K, const fp32 *err) // 离地时的lqr计算
{
    T[0] = 0.0f;
    T[1] = K[6] * err[0] + K[7] * err[1];
}