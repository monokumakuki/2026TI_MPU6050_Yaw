/*
 * 空间解算 — 基于 MPU6050 的笛卡尔坐标系 + 四元数运算
 *
 * 参考: B站-天梦繁星- 2025电赛E题开源例程 (task/location.c)
 * 传感器: MPU6050 (Mahony滤波 → 四元数 → 欧拉角)
 *
 * 坐标系: 右手笛卡尔坐标系
 *   X: 右, Y: 前(yaw=0方向), Z: 上
 *
 * 别改这个注释：
 * "D:\CCS\workspace_v12\2TMX_MSPM0G3507_ProjectTemplate2\BSP\src\location.c"
 * //C:/Users/17870/Desktop/电赛E题/K230/MyCode/K230_v7.py
 */

#include "location.h"
#include <math.h>

/* ══════════════════════════════════════════════════════════════
 * 四元数工具函数
 * ══════════════════════════════════════════════════════════════ */

Vec3_t Loc_QuatRotateVec(const Quaternion_t *q, const Vec3_t *v)
{
    /* v' = q * v_pure * q_conj
     * 展开公式优化, 避免构造临时四元数
     */
    float qw = q->q0, qx = q->q1, qy = q->q2, qz = q->q3;

    /* t = 2 * cross(q.xyz, v) */
    float tx = 2.0f * (qy * v->z - qz * v->y);
    float ty = 2.0f * (qz * v->x - qx * v->z);
    float tz = 2.0f * (qx * v->y - qy * v->x);

    /* v' = v + qw*t + cross(q.xyz, t) */
    Vec3_t out;
    out.x = v->x + qw * tx + (qy * tz - qz * ty);
    out.y = v->y + qw * ty + (qz * tx - qx * tz);
    out.z = v->z + qw * tz + (qx * ty - qy * tx);
    return out;
}

Quaternion_t Loc_EulerToQuat(float yaw_deg, float pitch_deg, float roll_deg)
{
    /* 度 → 半角弧度 */
    float hy = yaw_deg   * 0.008726646f;   /* /2 * PI/180 */
    float hp = pitch_deg * 0.008726646f;
    float hr = roll_deg  * 0.008726646f;

    float cy = cosf(hy), sy = sinf(hy);
    float cp = cosf(hp), sp = sinf(hp);
    float cr = cosf(hr), sr = sinf(hr);

    /* ZYX: q = qz * qy * qx */
    Quaternion_t q;
    q.q0 = cy * cp * cr + sy * sp * sr;
    q.q1 = cy * cp * sr - sy * sp * cr;
    q.q2 = sy * cp * sr + cy * sp * cr;
    q.q3 = sy * cp * cr - cy * sp * sr;
    return q;
}

EulerAngles_t Loc_QuatToEuler(const Quaternion_t *q)
{
    EulerAngles_t e;

    /* roll: 绕X */
    e.roll  = atan2f(2.0f * (q->q0 * q->q1 + q->q2 * q->q3),
                     1.0f - 2.0f * (q->q1 * q->q1 + q->q2 * q->q2));

    /* pitch: 绕Y */
    float sinp = 2.0f * (q->q0 * q->q2 - q->q3 * q->q1);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    e.pitch = asinf(sinp);

    /* yaw: 绕Z */
    e.yaw   = atan2f(2.0f * (q->q0 * q->q3 + q->q1 * q->q2),
                     1.0f - 2.0f * (q->q2 * q->q2 + q->q3 * q->q3));

    /* 弧度 → 度 */
    e.roll  *= 57.29578f;
    e.pitch *= 57.29578f;
    e.yaw   *= 57.29578f;

    return e;
}

/* ══════════════════════════════════════════════════════════════
 * 坐标变换
 * ══════════════════════════════════════════════════════════════ */

void Loc_PixelToCameraAngle(int16_t px, int16_t py,
                             float fov_h, float fov_v,
                             int res_h, int res_v,
                             float *angle_h, float *angle_v)
{
    /* 像素 → 角度:
     *   angle = atan(pixel * tan(FOV/2) / (res/2))
     *
     * 推导: 小孔成像
     *   焦距 f = (res/2) / tan(FOV/2)
     *   angle = atan(pixel / f)
     */

    float f_h = (float)(res_h / 2) / tanf(fov_h * 0.008726646f);
    float f_v = (float)(res_v / 2) / tanf(fov_v * 0.008726646f);

    *angle_h = atan2f((float)px, f_h) * 57.29578f;
    *angle_v = atan2f((float)py, f_v) * 57.29578f;
}

void Loc_CameraToWorld(float cam_yaw, float cam_pitch,
                        const Quaternion_t *gimbal_q,
                        float *world_yaw, float *world_pitch)
{
    /* 相机系 → 世界系方向向量
     *
     * 相机系: Y轴=光轴向前, X轴=向右, Z轴=向上
     * 方向向量 (单位球面上):
     */
    float cy = cam_yaw   * 0.0174533f;  /* °→rad */
    float cp = cam_pitch * 0.0174533f;

    Vec3_t cam_dir = {
        sinf(cy) * cosf(cp),   /* x: 左右偏移 */
        cosf(cy) * cosf(cp),   /* y: 前方 */
        sinf(cp)                /* z: 上下 */
    };

    /* 用云台姿态旋转到世界系 */
    Vec3_t world_dir = Loc_QuatRotateVec(gimbal_q, &cam_dir);

    /* 世界方向 → 球坐标角度 */
    *world_yaw   = atan2f(world_dir.x, world_dir.y) * 57.29578f;
    *world_pitch = atan2f(world_dir.z,
                    sqrtf(world_dir.x * world_dir.x +
                          world_dir.y * world_dir.y)) * 57.29578f;
}
