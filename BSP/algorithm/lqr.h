#ifndef DAMIAO_LQR_H
#define DAMIAO_LQR_H

#include "typedef.h"

float LQR_K_calc(const float *coe, float len);
void LQR_Calc(fp32 *T, const fp32 *K, const fp32 *err);
void LQR_Calc_air(fp32 *T, const fp32 *K, const fp32 *err);

#endif //DAMIAO_LQR_H