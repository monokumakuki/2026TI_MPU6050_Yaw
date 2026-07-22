#ifndef _PARAM_STORE_H
#define _PARAM_STORE_H

#include <stdint.h>
#include <stdbool.h>

/******************************************************************
 * Flash 参数存储模块
 *
 * 存储格式（24 字节，6 个 32-bit Word）：
 *   Word 0: [31:0]   Magic = 0x4D534B50 ("MSKP")
 *   Word 1: [47:32]  KP,       [63:48]  KD
 *   Word 2: [79:64]  LostGain, [95:80]  BaseSpeed
 *   Word 3: [111:96] Laps,     [127:112] PivotTime
 *   Word 4: [135:128] MpuMode, [143:136] TurnSpeed(80~250)
 *   Word 4+5: 保留
 ******************************************************************/

bool Param_Load(int16_t *kp, int16_t *kd, int16_t *lost_gain,
                int16_t *base_speed, int16_t *laps, int16_t *pivot_time,
                uint8_t *mpu_mode, int16_t *pivot_speed,
                int16_t *yaw_kp10, int16_t *yaw_rate100,
                int16_t *mpu_mix, int16_t *yaw_limit,
                int16_t *yaw_lock_count, int16_t *curve_rate100);

bool Param_Save(int16_t kp, int16_t kd, int16_t lost_gain,
                int16_t base_speed, int16_t laps, int16_t pivot_time,
                uint8_t mpu_mode, int16_t pivot_speed,
                int16_t yaw_kp10, int16_t yaw_rate100,
                int16_t mpu_mix, int16_t yaw_limit,
                int16_t yaw_lock_count, int16_t curve_rate100);

#endif
