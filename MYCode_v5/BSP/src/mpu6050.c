#include "mpu6050.h"
#include "board.h"
#include <math.h>

/* 当前量程配置, 默认 ±2g / ±250dps */
static float s_accel_sens = MPU6050_ACCEL_SENS_2G;
static float s_gyro_sens  = MPU6050_GYRO_SENS_250;

/* 校准参数 */
static MPU6050_Calib_t s_calib = {0};

/* ================ I2C 底层读写 ================ */

void MPU6050_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t txData[2];
    txData[0] = reg;
    txData[1] = val;

    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));

    DL_I2C_fillControllerTXFIFO(MPU6050_INST, txData, 2);
    DL_I2C_startControllerTransfer(MPU6050_INST, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS));
    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
}

uint8_t MPU6050_ReadReg(uint8_t reg)
{
    uint8_t val;

    /* 发送寄存器地址 */
    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));

    DL_I2C_fillControllerTXFIFO(MPU6050_INST, &reg, 1);
    DL_I2C_startControllerTransfer(MPU6050_INST, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS));
    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));

    /* 读取1字节数据 */
    DL_I2C_startControllerTransfer(MPU6050_INST, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, 1);

    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS));
    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));

    val = DL_I2C_receiveControllerData(MPU6050_INST);

    return val;
}

/* 读取多个连续寄存器 */
static void MPU6050_ReadRegs(uint8_t reg, uint8_t len, uint8_t *buf)
{
    /* 发送起始寄存器地址 */
    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));

    DL_I2C_fillControllerTXFIFO(MPU6050_INST, &reg, 1);
    DL_I2C_startControllerTransfer(MPU6050_INST, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS));
    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));

    /* 读取 len 字节数据 */
    DL_I2C_startControllerTransfer(MPU6050_INST, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);

    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS));
    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));

    for (uint8_t i = 0; i < len; i++) {
        buf[i] = DL_I2C_receiveControllerData(MPU6050_INST);
    }
}

/* ================ 功能接口 ================ */

int MPU6050_Init(void)
{
    uint8_t who_am_i;

    /* 唤醒 MPU6050, 取消睡眠模式 */
    MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00);
    delay_ms(10);

    /* 验证设备ID */
    who_am_i = MPU6050_ReadWhoAmI();
    if (who_am_i != 0x68 && who_am_i != 0x70)  /* 兼容 MPU6050 & MPU6500 */
        return -1;

    /* 采样率分频: 采样率 = 1kHz / (1 + SMPLRT_DIV), 0x00 = 1kHz */
    MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x00);

    /* 低通滤波: DLPF_CFG=3, ~44Hz 带宽 */
    MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x03);

    /* 陀螺仪量程: ±250dps */
    MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_250DPS);
    s_gyro_sens = MPU6050_GYRO_SENS_250;

    /* 加速度计量程: ±2g */
    MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_2G);
    s_accel_sens = MPU6050_ACCEL_SENS_2G;

    /* 电源管理: 使用 X 轴陀螺仪时钟, 更稳定 */
    MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x01);

    return 0;
}

uint8_t MPU6050_ReadWhoAmI(void)
{
    return MPU6050_ReadReg(MPU6050_REG_WHO_AM_I);
}

void MPU6050_ReadAccelRaw(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];
    MPU6050_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, 6, buf);

    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
}

void MPU6050_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];
    MPU6050_ReadRegs(MPU6050_REG_GYRO_XOUT_H, 6, buf);

    *gx = (int16_t)((buf[0] << 8) | buf[1]);
    *gy = (int16_t)((buf[2] << 8) | buf[3]);
    *gz = (int16_t)((buf[4] << 8) | buf[5]);
}

int16_t MPU6050_ReadTempRaw(void)
{
    uint8_t buf[2];
    MPU6050_ReadRegs(MPU6050_REG_TEMP_OUT_H, 2, buf);

    return (int16_t)((buf[0] << 8) | buf[1]);
}

void MPU6050_ReadData(MPU6050_Data_t *data)
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;

    /* 与可运行的 Test_MPU6050 保持一致：每次读取不超过8字节接收FIFO容量。 */
    MPU6050_ReadAccelRaw(&ax, &ay, &az);
    MPU6050_ReadGyroRaw(&gx, &gy, &gz);
    temp = MPU6050_ReadTempRaw();

    data->accel_x = (float)ax / s_accel_sens;
    data->accel_y = (float)ay / s_accel_sens;
    data->accel_z = (float)az / s_accel_sens;

    data->gyro_x = (float)gx / s_gyro_sens;
    data->gyro_y = (float)gy / s_gyro_sens;
    data->gyro_z = (float)gz / s_gyro_sens;

    /* 温度换算公式: Temperature = (raw / 340.0) + 36.53 */
    data->temperature = (float)temp / 340.0f + 36.53f;
}

/* ================ 校准与姿态 ================ */

void MPU6050_Calibrate(uint16_t samples)
{
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int16_t ax, ay, az, gx, gy, gz;

    /* 丢弃前10个不稳定采样 */
    for (uint16_t i = 0; i < 10; i++) {
        MPU6050_ReadAccelRaw(&ax, &ay, &az);
        MPU6050_ReadGyroRaw(&gx, &gy, &gz);
        delay_ms(2);
    }

    /* 累加采样 */
    for (uint16_t i = 0; i < samples; i++) {
        MPU6050_ReadAccelRaw(&ax, &ay, &az);
        MPU6050_ReadGyroRaw(&gx, &gy, &gz);

        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;

        delay_ms(2);
    }

    /* 陀螺仪零偏: 静止时均值即为零偏 */
    s_calib.gyro_x_offset = (int16_t)(gx_sum / samples);
    s_calib.gyro_y_offset = (int16_t)(gy_sum / samples);
    s_calib.gyro_z_offset = (int16_t)(gz_sum / samples);

    /* 加速度计零偏: X/Y 静止时均值即为零偏, Z轴以1g为参考 */
    s_calib.accel_x_offset = (int16_t)(ax_sum / samples);
    s_calib.accel_y_offset = (int16_t)(ay_sum / samples);
    /* Z轴: 水平放置时 Z 应为 +1g, 灵敏度16384 LSB/g → 期望值16384 */
    s_calib.accel_z_offset = (int16_t)(az_sum / samples) - (int16_t)s_accel_sens;
}

void MPU6050_ReadDataCalibrated(MPU6050_Data_t *data)
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;

    MPU6050_ReadAccelRaw(&ax, &ay, &az);
    MPU6050_ReadGyroRaw(&gx, &gy, &gz);
    temp = MPU6050_ReadTempRaw();

    /* 减去零偏 */
    ax -= s_calib.accel_x_offset;
    ay -= s_calib.accel_y_offset;
    az -= s_calib.accel_z_offset;
    gx -= s_calib.gyro_x_offset;
    gy -= s_calib.gyro_y_offset;
    gz -= s_calib.gyro_z_offset;

    data->accel_x = (float)ax / s_accel_sens;
    data->accel_y = (float)ay / s_accel_sens;
    data->accel_z = (float)az / s_accel_sens;

    data->gyro_x = (float)gx / s_gyro_sens;
    data->gyro_y = (float)gy / s_gyro_sens;
    data->gyro_z = (float)gz / s_gyro_sens;

    data->temperature = (float)temp / 340.0f + 36.53f;
}

void MPU6050_GetAttitude(const MPU6050_Data_t *data, MPU6050_Attitude_t *att)
{
    /* 根据加速度计算横滚角和俯仰角 */
    att->roll  = atan2f(data->accel_y, data->accel_z) * 180.0f / 3.14159265f;
    att->pitch = atan2f(-data->accel_x,
        sqrtf(data->accel_y * data->accel_y + data->accel_z * data->accel_z))
        * 180.0f / 3.14159265f;
}

const MPU6050_Calib_t* MPU6050_GetCalibParams(void)
{
    return &s_calib;
}

/* ==================== Mahony 四元数姿态解算 ==================== */

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float mahony_kp = 2.0f, mahony_ki = 0.005f;
static float mahony_i[3] = {0};
static float cached_roll = 0.0f, cached_pitch = 0.0f, cached_yaw = 0.0f;
static float yaw_offset = 0.0f;
static MPU6050_Data_t s_latest_data = {0};
static uint32_t s_last_update_ms = 0U;

/* 自动周期只用于兼容旧接口。实际周期来自 1ms SysTick，而不是假定每次调用都是1ms。 */
static float MPU6050_GetAutoDt(void)
{
    uint32_t now = Board_Millis();
    uint32_t elapsed = now - s_last_update_ms;
    s_last_update_ms = now;
    if (elapsed == 0U) return 0.0f;
    if (elapsed > 50U) elapsed = 50U; /* 按键阻塞或暂停后不积分一个异常大步长 */
    return (float)elapsed * 0.001f;
}

void MPU6050_UpdateTimeReset(void)
{
    s_last_update_ms = Board_Millis();
}

/* 快速 invSqrt */
static float invSqrt(float x)
{
    union { float f; int32_t i; } u;
    float halfx = 0.5f * x;
    u.f = x;
    u.i = 0x5f3759df - (u.i >> 1);
    float y = u.f;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void MPU6050_QuaternionUpdate(void)
{
    MPU6050_QuaternionUpdateDt(MPU6050_GetAutoDt());
}

void MPU6050_QuaternionUpdateDt(float dt)
{
    MPU6050_Data_t data;
    MPU6050_ReadDataCalibrated(&data);
    s_latest_data = data;

    if (dt <= 0.0f) return;
    if (dt > 0.05f) dt = 0.05f;

    /* 归一化加速度 */
    float ax = data.accel_x, ay = data.accel_y, az = data.accel_z;
    float acc_norm_sq = ax * ax + ay * ay + az * az;
    if (acc_norm_sq < 0.01f) return;
    float norm = invSqrt(acc_norm_sq);
    ax *= norm; ay *= norm; az *= norm;

    /* 陀螺仪 rad/s + 死区 */
    float gx = data.gyro_x * 0.0174533f;
    float gy = data.gyro_y * 0.0174533f;
    float gz = data.gyro_z * 0.0174533f;
    {
        const float dz = 0.25f * 0.0174533f;
        if (gx > -dz && gx < dz) gx = 0.0f;
        if (gy > -dz && gy < dz) gy = 0.0f;
        if (gz > -dz && gz < dz) gz = 0.0f;
    }

    /* 推算重力方向 */
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* 叉积误差 */
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    /* PI 修正 + 积分限幅 */
    mahony_i[0] += ex * mahony_ki * dt;
    mahony_i[1] += ey * mahony_ki * dt;
    mahony_i[2] += ez * mahony_ki * dt;
    {
        const float lim = 0.3f;
        if (mahony_i[0] > lim) mahony_i[0] = lim;
        if (mahony_i[0] < -lim) mahony_i[0] = -lim;
        if (mahony_i[1] > lim) mahony_i[1] = lim;
        if (mahony_i[1] < -lim) mahony_i[1] = -lim;
        if (mahony_i[2] > lim) mahony_i[2] = lim;
        if (mahony_i[2] < -lim) mahony_i[2] = -lim;
    }
    gx += mahony_kp * ex + mahony_i[0];
    gy += mahony_kp * ey + mahony_i[1];
    gz += mahony_kp * ez + mahony_i[2];

    /* 四元数积分: q += 0.5*dt * (q ⊗ ω) */
    {
        float hdt = 0.5f * dt;
        float qa = q0, qb = q1, qc = q2, qd = q3;
        q0 += hdt * (-qb * gx - qc * gy - qd * gz);
        q1 += hdt * ( qa * gx + qc * gz - qd * gy);
        q2 += hdt * ( qa * gy - qb * gz + qd * gx);
        q3 += hdt * ( qa * gz + qb * gy - qc * gx);
    }

    /* 归一化 */
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;

    /* 四元数 → 欧拉角 */
    cached_roll  = atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1,
                           2.0f * q0 * q0 + 2.0f * q3 * q3 - 1.0f) * 57.29578f;
    {
        float sp = 2.0f * q0 * q2 - 2.0f * q1 * q3;
        if (sp > 1.0f) sp = 1.0f; if (sp < -1.0f) sp = -1.0f;
        cached_pitch = -asinf(sp) * 57.29578f;
    }
    cached_yaw = atan2f(2.0f * q1 * q2 + 2.0f * q0 * q3,
                         2.0f * q0 * q0 + 2.0f * q1 * q1 - 1.0f) * 57.29578f;
}

/**
 * @brief 轻量级更新 — 只读陀螺仪做四元数积分, 无加速度计修正
 * @note  I2C 一次 6 字节读取, ~0.15ms, 巡线时不阻塞
 *        yaw 会缓慢漂移(~0.5°/min), 进菜单页用完整版修正
 */
void MPU6050_QuaternionUpdateFast(void)
{
    MPU6050_QuaternionUpdateFastDt(MPU6050_GetAutoDt());
}

void MPU6050_QuaternionUpdateFastDt(float dt)
{
    int16_t raw_gx, raw_gy, raw_gz;
    MPU6050_ReadGyroRaw(&raw_gx, &raw_gy, &raw_gz);

    if (dt <= 0.0f) return;
    if (dt > 0.05f) dt = 0.05f;

    /* 减去校准零偏 + 转 rad/s + 死区 */
    float gx = (float)(raw_gx - s_calib.gyro_x_offset) / s_gyro_sens * 0.0174533f;
    float gy = (float)(raw_gy - s_calib.gyro_y_offset) / s_gyro_sens * 0.0174533f;
    float gz = (float)(raw_gz - s_calib.gyro_z_offset) / s_gyro_sens * 0.0174533f;
    {
        const float dz = 0.25f * 0.0174533f;
        if (gx > -dz && gx < dz) gx = 0.0f;
        if (gy > -dz && gy < dz) gy = 0.0f;
        if (gz > -dz && gz < dz) gz = 0.0f;
    }

    /* 四元数积分 (无加速度修正) */
    {
        float hdt = 0.5f * dt;
        float qa = q0, qb = q1, qc = q2, qd = q3;
        q0 += hdt * (-qb * gx - qc * gy - qd * gz);
        q1 += hdt * ( qa * gx + qc * gz - qd * gy);
        q2 += hdt * ( qa * gy - qb * gz + qd * gx);
        q3 += hdt * ( qa * gz + qb * gy - qc * gx);
    }

    /* 归一化 */
    float norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;

    /* → 欧拉角 */
    cached_roll  = atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1,
                           2.0f * q0 * q0 + 2.0f * q3 * q3 - 1.0f) * 57.29578f;
    {
        float sp = 2.0f * q0 * q2 - 2.0f * q1 * q3;
        if (sp > 1.0f) sp = 1.0f; if (sp < -1.0f) sp = -1.0f;
        cached_pitch = -asinf(sp) * 57.29578f;
    }
    cached_yaw = atan2f(2.0f * q1 * q2 + 2.0f * q0 * q3,
                         2.0f * q0 * q0 + 2.0f * q1 * q1 - 1.0f) * 57.29578f;
}

void MPU6050_GetEuler(EulerAngles_t *euler)
{
    euler->roll  = cached_roll;
    euler->pitch = cached_pitch;
    euler->yaw   = cached_yaw - yaw_offset;
}

void MPU6050_GetLatestData(MPU6050_Data_t *data)
{
    if (data != 0) *data = s_latest_data;
}

void MPU6050_QuaternionReset(void)
{
    yaw_offset = 0.0f;
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    cached_roll = 0.0f;
    cached_pitch = 0.0f;
    cached_yaw = 0.0f;
    mahony_i[0] = mahony_i[1] = mahony_i[2] = 0.0f;
    MPU6050_UpdateTimeReset();
}

/* ==================== 互补滤波航向角 ==================== */

/* 互补滤波系数：越大越信任陀螺仪，越小越信任加速度计
 * 典型值0.96~0.98，MPU6050陀螺仪短期精度高，取0.98 */
static float s_comp_alpha = 0.98f;

/* 当前航向角(度)，顺时针为正 */
static float s_yaw = 0.0f;
static uint32_t s_comp_last_ms = 0U;

/******************************************************************
 * 函 数 名 称：MPU6050_ComplementaryFilterUpdate
 * 函 数 说 明：更新互补滤波航向角
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：调用间隔由 Board_Millis() 实测
 *             陀螺仪Z轴积分提供短期相对 yaw
 *             六轴 MPU6050 无法用加速度计消除 yaw 长期漂移
******************************************************************/
void MPU6050_ComplementaryFilterUpdate(void)
{
    MPU6050_Data_t data;
    uint32_t now = Board_Millis();
    uint32_t elapsed = now - s_comp_last_ms;
    s_comp_last_ms = now;
    if (elapsed == 0U) return;
    if (elapsed > 50U) elapsed = 50U;

    MPU6050_ReadDataCalibrated(&data);

    /* 六轴 MPU6050 的加速度计不能观测绕重力轴的 yaw。
     * 因此按真实 dt 积分 Z 轴，不再用 atan2(accel_y, accel_x) 错误修正 yaw。 */
    s_yaw += data.gyro_z * ((float)elapsed * 0.001f);
    while (s_yaw > 180.0f) s_yaw -= 360.0f;
    while (s_yaw < -180.0f) s_yaw += 360.0f;
}

/******************************************************************
 * 函 数 名 称：MPU6050_GetYaw
 * 函 数 说 明：获取互补滤波后的航向角
 * 函 数 形 参：无
 * 函 数 返 回：float 航向角(度)，顺时针为正
******************************************************************/
float MPU6050_GetYaw(void)
{
    return s_yaw;
}

/******************************************************************
 * 函 数 名 称：MPU6050_ResetYaw
 * 函 数 说 明：重置航向角为0
******************************************************************/
void MPU6050_ResetYaw(void)
{
    s_yaw = 0.0f;
    s_comp_last_ms = Board_Millis();
}

/******************************************************************
 * 函 数 名 称：MPU6050_SetComplementaryAlpha
 * 函 数 说 明：设置互补滤波系数
 * 函 数 形 参：alpha - 滤波系数(0.0~1.0)
 * 函 数 返 回：无
 * 备       注：越大越信任陀螺仪，越小越信任加速度计
 *             典型值0.96~0.98，默认0.98
******************************************************************/
void MPU6050_SetComplementaryAlpha(float alpha)
{
    /* 为兼容旧调用保留该参数；当前纯六轴 yaw 解算不使用加速度计航向融合。 */
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    s_comp_alpha = alpha;
}

/* ==================== 角度PID转弯控制 ==================== */

#include "PID.h"
#include "line.h"

/* 角度PID实例 */
static PID_t s_angle_pid;

/* 目标航向角 */
static float s_angle_target = 0.0f;

/******************************************************************
 * 函 数 名 称：MPU6050_AnglePID_Init
 * 函 数 说 明：初始化角度PID控制器
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：角度PID默认参数：
 *             KP=5.0  角度误差1度→输出5的速度差
 *             KI=0.1  消除稳态角度偏差
 *             KD=0.5  抑制转弯振荡
 *             输出范围-500~+500，对应速度差值
******************************************************************/
void MPU6050_AnglePID_Init(void)
{
    PID_Init(&s_angle_pid, 5.0f, 0.1f, 0.5f, -500.0f, 500.0f, 200.0f);
    s_angle_target = 0.0f;
}

/******************************************************************
 * 函 数 名 称：MPU6050_AnglePID_SetTarget
 * 函 数 说 明：设置目标航向角
 * 函 数 形 参：target_angle - 目标角度(度)，相对于当前偏移
 *             正值=顺时针转，负值=逆时针转
 * 函 数 返 回：无
******************************************************************/
void MPU6050_AnglePID_SetTarget(float target_angle)
{
    s_angle_target = target_angle;
    PID_Reset(&s_angle_pid);
}

/******************************************************************
 * 函 数 名 称：MPU6050_AnglePID_Calculate
 * 函 数 说 明：执行一次角度PID计算
 * 函 数 形 参：无
 * 函 数 返 回：float 角度修正量
 *             正值表示右转修正，负值表示左转修正
 * 备       注：返回值可直接叠加到左右轮速度差上：
 *             left_speed  = base_speed - output
 *             right_speed = base_speed + output
******************************************************************/
float MPU6050_AnglePID_Calculate(void)
{
    return PID_Calculate(&s_angle_pid, s_angle_target, s_yaw);
}

/******************************************************************
 * 函 数 名 称：MPU6050_AnglePID_Reset
 * 函 数 说 明：重置角度PID状态
******************************************************************/
void MPU6050_AnglePID_Reset(void)
{
    PID_Reset(&s_angle_pid);
    s_angle_target = 0.0f;
}

/******************************************************************
 * 函 数 名 称：MPU6050_AnglePID_SetParams
 * 函 数 说 明：修改角度PID参数
******************************************************************/
void MPU6050_AnglePID_SetParams(float kp, float ki, float kd)
{
    PID_SetParams(&s_angle_pid, kp, ki, kd);
}

/******************************************************************
 * 函 数 名 称：MPU6050_AnglePID_GetError
 * 函 数 说 明：获取当前航向角误差
 * 函 数 形 参：无
 * 函 数 返 回：float 角度误差(度)
 *             正值表示还需右转，负值表示还需左转
******************************************************************/
float MPU6050_AnglePID_GetError(void)
{
    return s_angle_target - s_yaw;
}

/******************************************************************
 * 函 数 名 称：MPU6050_TurnAngle
 * 函 数 说 明：精确转弯指定角度（阻塞式）
 * 函 数 形 参：angle      - 转弯角度(度)，正值右转，负值左转
 *             base_speed - 转弯时的基础速度(0~999)
 * 函 数 返 回：无
 * 备       注：阻塞式执行，直到达到目标角度后停车
 *             调用前需确保MPU6050已初始化和校准
 *             内部自动记录起始航向角并设置目标
******************************************************************/
void MPU6050_TurnAngle(float angle, uint32_t base_speed)
{
    /* 记录起始航向角 */
    float start_yaw = s_yaw;

    /* 设置目标角度：当前角度 + 偏移量 */
    s_angle_target = start_yaw + angle;
    PID_Reset(&s_angle_pid);

    /* 转弯执行循环 */
    while (1)
    {
        /* 更新航向角 */
        MPU6050_ComplementaryFilterUpdate();

        /* PID计算修正量 */
        float output = MPU6050_AnglePID_Calculate();

        /* 映射到左右轮差速：
         * output > 0 → 需右转 → 左轮加速，右轮减速
         * output < 0 → 需左转 → 左轮减速，右轮加速 */
        int16_t left_speed  = (int16_t)base_speed - (int16_t)output;
        int16_t right_speed = (int16_t)base_speed + (int16_t)output;

        /* 限幅 */
        if (left_speed > 999)  left_speed = 999;
        if (left_speed < -999) left_speed = -999;
        if (right_speed > 999)  right_speed = 999;
        if (right_speed < -999) right_speed = -999;

        /* 判断是否到达目标角度（误差<1度） */
        float error = s_angle_target - s_yaw;
        if (error < 0.0f) error = -error;
        if (error < 1.0f)
        {
            Car_Stop();
            break;
        }

        /* 输出到电机 */
        Car_SetMotors(left_speed, right_speed);

        delay_ms(1);
    }
}
