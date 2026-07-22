#ifndef __LOCATION_H__
#define __LOCATION_H__

#include <stdint.h>
#include "mpu6050.h"  /* Quaternion_t, EulerAngles_t */

/******************************************************************
 * 空间解算框架 — 笛卡尔坐标系 + 云台姿态变换
 *
 * 参考: B站-天梦繁星- 2025电赛E题开源例程 (task/location.c)
 * 传感器: MPU6050 (已有 mpu6050.c 提供四元数/欧拉角)
 *
 * === 坐标系 (右手笛卡尔坐标系) ===
 *   X轴: 水平向右 (从云台后方看)
 *   Y轴: 水平向前 (初始朝向, yaw=0°的方向)
 *   Z轴: 垂直向上
 *
 * === 欧拉角 (ZYX 顺规) ===
 *   yaw:   绕Z轴 (偏航, 水平面内旋转)  — 由陀螺仪Z轴积分得到
 *   pitch: 绕Y轴 (俯仰, 抬头为正)
 *   roll:  绕X轴 (横滚, 右倾为正)
 *
 * === 四元数 q = w + xi + yj + zk ===
 *   MPU6050 的 Mahony 滤波器已提供: MPU6050_GetQuaternion()
 *   四元数 → 欧拉角: MPU6050_GetEuler()
 *
 * TODO: 像素偏差 → 世界角度 → 云台控制 (用户要求暂缓)
 ******************************************************************/

/* 三维向量 */
typedef struct {
    float x, y, z;
} Vec3_t;

/* 空间位姿: 位置(暂不用) + 姿态四元数 */
typedef struct {
    Vec3_t        pos;
    Quaternion_t  q;       /* 使用 mpu6050.h 的 Quaternion_t */
} Pose_t;

/* ══════════════════════════════════════════════════════════════
 * 四元数工具函数 (补充 mpu6050 已有的 Mahony/GetEuler)
 * ══════════════════════════════════════════════════════════════ */

/* 用四元数旋转一个三维向量: out = q * v * q^-1 */
Vec3_t Loc_QuatRotateVec(const Quaternion_t *q, const Vec3_t *v);

/* 欧拉角(°) → 四元数 */
Quaternion_t Loc_EulerToQuat(float yaw_deg, float pitch_deg, float roll_deg);

/* 四元数 → 欧拉角(°) — 与 MPU6050_GetEuler 等效, 独立实现 */
EulerAngles_t Loc_QuatToEuler(const Quaternion_t *q);

/* ══════════════════════════════════════════════════════════════
 * 坐标变换 — 相机像素 → 世界角度
 *
 * 推导 (参考 location.c 的 convert_xy_to_angle):
 *   1. 像素偏差 → 相机系方向向量
 *      cam_dx = tan(px * tan(fov_h/2) / (res_h/2))
 *   2. 相机系 → 云台系 → 世界系 (通过姿态四元数旋转)
 *      world_dir = q_gimbal * cam_dir * q_gimbal^-1
 *   3. 世界系方向 → 方位角
 *      yaw = atan2(world_dir.x, world_dir.y)
 *      pitch = atan2(world_dir.z, sqrt(x² + y²))
 *
 * ══════════════════════════════════════════════════════════════ */

/* 像素偏差 → 角度偏移 (忽略云台姿态, 纯相机几何)
 *
 * 参数:
 *   px, py:      像素偏差 (K230发来的)
 *   fov_h, fov_v: 相机视场角 (°)
 *   res_h, res_v: 相机分辨率
 *   angle_h, angle_v: 输出相机系角度 (°)
 *
 * 公式: angle = atan(pixel * tan(FOV/2) / (res/2))  [弧度→度]
 */
void Loc_PixelToCameraAngle(int16_t px, int16_t py,
                             float fov_h, float fov_v,
                             int res_h, int res_v,
                             float *angle_h, float *angle_v);

/* 相机系角度 + 云台姿态 → 世界系角度 (完整空间解算)
 *
 * 参数:
 *   cam_yaw, cam_pitch: 相机系目标角度 (°), 由 Loc_PixelToCameraAngle 得到
 *   gimbal_q:           云台当前姿态四元数 (来自 MPU6050_GetQuaternion)
 *   world_yaw, world_pitch: 输出世界系目标角度 (°)
 *
 * NOTE: 当前仅做框架, 云台姿态=水平时相机系=世界系
 *       TODO: 完整实现需要标定相机FOV和云台安装角
 */
void Loc_CameraToWorld(float cam_yaw, float cam_pitch,
                        const Quaternion_t *gimbal_q,
                        float *world_yaw, float *world_pitch);

#endif
