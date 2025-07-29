#include <SOGI.h>

SOGI_TypeDef sogi;

void SOGI_Init(SOGI_TypeDef *sogi, float w0, float T, float w1)
{
    sogi->v_alpha = 0.0f;
    sogi->v_beta = 0.0f;
    sogi->k = 1.414f; // 初始增益
    sogi->w0 = w0; // 角频率
    sogi->T = T; // 定时器周期

    sogi->q = 0.0f; // 初始化q轴分量
    sogi->d = 0.0f; // 初始化d轴分量
    sogi->w1 = w1; // 初始化w1
    sogi->theta = 0.0f; // 初始化相位角

    sogi->kp = 1.0f; // 比例增益
    sogi->ki = 0.5f; // 积分增益
    sogi->err_prev = 0.0f;
    sogi->err = 0.0f; // 初始化误差
}

void SOGI_Update(SOGI_TypeDef *sogi, float v_in)
{
    sogi->v_alpha += (((v_in - sogi->v_alpha) * sogi->k) - sogi->v_beta) * sogi->w0 * sogi->T;
    sogi->v_beta += sogi->v_alpha * sogi->w0 * sogi->T;
}

void SOGI_PLL(SOGI_TypeDef *sogi)
{
    sogi->q = arm_cos_f32(sogi->theta) * sogi->v_beta - arm_sin_f32(sogi->theta) * sogi->v_alpha;
    sogi->err = 0 - sogi->q;
    // 增量PI控制
    float delta_w = sogi->kp * (sogi->err - sogi->err_prev) + sogi->ki * sogi->T * sogi->err;
    sogi->w1 += delta_w;

    if (sogi->w1 > 130.0f * PI) sogi->w1 = 130.0f * PI;
    else if (sogi->w1 < 70.0f * PI) sogi->w1 = 70.0f * PI;

    sogi->theta += sogi->w1 * sogi->T;

    if (sogi->theta >= 2.0f * PI) sogi->theta -= 2.0f * PI;
    else if (sogi->theta < 0.0f) sogi->theta += 2.0f * PI;

    sogi->err_prev = sogi->err;
}


