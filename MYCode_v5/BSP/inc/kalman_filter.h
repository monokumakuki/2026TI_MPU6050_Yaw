#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#include <stdint.h>

/**
 * @brief 一维卡尔曼滤波器 — 用于角度平滑
 *
 * 用法:
 *   KalmanFilter kf;
 *   Kalman_Init(&kf, 0.05f, 1.0f);    // Q小=平滑, R=信任测量
 *   float smooth = Kalman_Update(&kf, raw_angle);
 */
typedef struct {
    float Q;    // 过程噪声协方差 (小=更平滑, 大=更跟测量)
    float R;    // 测量噪声协方差 (小=更信测量, 大=更平滑)
    float P;    // 估计误差协方差
    float K;    // 卡尔曼增益
    float X;    // 最优估计值
} KalmanFilter;

void Kalman_Init(KalmanFilter* kf, float Q, float R);
float Kalman_Update(KalmanFilter* kf, float measurement);

#endif
