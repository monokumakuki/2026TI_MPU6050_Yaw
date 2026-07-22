/*
 * 一维卡尔曼滤波器 — 参考 B站-天梦繁星- 电赛E题开源例程
 */
#include "kalman_filter.h"

void Kalman_Init(KalmanFilter* kf, float Q, float R)
{
    kf->Q = Q;
    kf->R = R;
    kf->P = 1.0f;
    kf->K = 0.0f;
    kf->X = 0.0f;
}

float Kalman_Update(KalmanFilter* kf, float measurement)
{
    /* 首次测量直接初始化 */
    if (kf->X == 0.0f) {
        kf->X = measurement;
        return measurement;
    }

    /* 预测: X不变, P累积Q */
    kf->P = kf->P + kf->Q;

    /* 更新: 卡尔曼增益 */
    kf->K = kf->P / (kf->P + kf->R);

    /* 融合测量值 */
    kf->X = kf->X + kf->K * (measurement - kf->X);

    /* 更新误差协方差 */
    kf->P = (1.0f - kf->K) * kf->P;

    return kf->X;
}
