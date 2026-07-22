#ifndef __MPU6050_H
#define __MPU6050_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* MPU6050 I2C 从机地址 (AD0=0) */
#define MPU6050_ADDR                0x68

/* MPU6050 寄存器地址 */
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_PWR_MGMT_2      0x6C
#define MPU6050_REG_WHO_AM_I        0x75

/* 加速度计量程 */
#define MPU6050_ACCEL_2G            0x00
#define MPU6050_ACCEL_4G            0x08
#define MPU6050_ACCEL_8G            0x10
#define MPU6050_ACCEL_16G           0x18

/* 陀螺仪量程 */
#define MPU6050_GYRO_250DPS         0x00
#define MPU6050_GYRO_500DPS         0x08
#define MPU6050_GYRO_1000DPS        0x10
#define MPU6050_GYRO_2000DPS        0x18

/* 加速度计灵敏度 (LSB/g) */
#define MPU6050_ACCEL_SENS_2G       16384.0f
#define MPU6050_ACCEL_SENS_4G       8192.0f
#define MPU6050_ACCEL_SENS_8G       4096.0f
#define MPU6050_ACCEL_SENS_16G      2048.0f

/* 陀螺仪灵敏度 (LSB/(deg/s)) */
#define MPU6050_GYRO_SENS_250       131.0f
#define MPU6050_GYRO_SENS_500       65.5f
#define MPU6050_GYRO_SENS_1000      32.8f
#define MPU6050_GYRO_SENS_2000      16.4f

/* MPU6050 数据结构体 */
typedef struct {
    float accel_x;      // 加速度 X (g)
    float accel_y;      // 加速度 Y (g)
    float accel_z;      // 加速度 Z (g)
    float gyro_x;       // 角速度 X (deg/s)
    float gyro_y;       // 角速度 Y (deg/s)
    float gyro_z;       // 角速度 Z (deg/s)
    float temperature;  // 温度 (°C)
} MPU6050_Data_t;

/* 四元数结构体 */
typedef struct {
    float q0;           // w (实部)
    float q1;           // x
    float q2;           // y
    float q3;           // z
} Quaternion_t;

/* 姿态角结构体 */
typedef struct {
    float roll;         // 横滚角 (度)
    float pitch;        // 俯仰角 (度)
} MPU6050_Attitude_t;

/* 欧拉角结构体 (Mahony 四元数输出) */
typedef struct {
    float roll;         // 横滚角 (度), -180~+180
    float pitch;        // 俯仰角 (度), -90~+90
    float yaw;          // 航向角 (度), -180~+180
} EulerAngles_t;

/* 校准参数结构体 */
typedef struct {
    int16_t accel_x_offset;
    int16_t accel_y_offset;
    int16_t accel_z_offset;  // Z轴校准后应为1g对应的原始值
    int16_t gyro_x_offset;
    int16_t gyro_y_offset;
    int16_t gyro_z_offset;
} MPU6050_Calib_t;

/**
 * @brief 初始化 MPU6050
 * @return 0 成功, -1 失败
 */
int MPU6050_Init(void);

/**
 * @brief 读取 WHO_AM_I 寄存器
 * @return 设备ID, 正常应为 0x68
 */
uint8_t MPU6050_ReadWhoAmI(void);

/**
 * @brief 读取原始加速度数据 (16位有符号)
 * @param ax 加速度X原始值
 * @param ay 加速度Y原始值
 * @param az 加速度Z原始值
 */
void MPU6050_ReadAccelRaw(int16_t *ax, int16_t *ay, int16_t *az);

/**
 * @brief 读取原始陀螺仪数据 (16位有符号)
 * @param gx 陀螺仪X原始值
 * @param gy 陀螺仪Y原始值
 * @param gz 陀螺仪Z原始值
 */
void MPU6050_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz);

/**
 * @brief 读取原始温度数据 (16位有符号)
 * @return 温度原始值
 */
int16_t MPU6050_ReadTempRaw(void);

/**
 * @brief 读取转换后的全部数据 (浮点, 带单位)
 * @param data 存放数据的结构体指针
 */
void MPU6050_ReadData(MPU6050_Data_t *data);

/**
 * @brief 向 MPU6050 写入单字节寄存器
 * @param reg 寄存器地址
 * @param val 写入值
 */
void MPU6050_WriteReg(uint8_t reg, uint8_t val);

/**
 * @brief 从 MPU6050 读取单字节寄存器
 * @param reg 寄存器地址
 * @return 读取值
 */
uint8_t MPU6050_ReadReg(uint8_t reg);

/**
 * @brief 上电自动校准零偏 (传感器需保持静止水平放置)
 * @param samples 校准采样次数, 默认200
 * @note 陀螺仪零偏取均值减去, 加速度X/Y零偏取均值减去,
 *       Z轴以1g为参考修正
 */
void MPU6050_Calibrate(uint16_t samples);

/**
 * @brief 读取校准后的数据 (已减零偏)
 * @param data 存放数据的结构体指针
 */
void MPU6050_ReadDataCalibrated(MPU6050_Data_t *data);

/**
 * @brief 计算姿态角 (基于校准后的加速度, 横滚/俯仰)
 * @param data 校准后的数据
 * @param att  输出姿态角
 */
void MPU6050_GetAttitude(const MPU6050_Data_t *data, MPU6050_Attitude_t *att);

/**
 * @brief 获取当前校准参数
 * @return 校准参数指针
 */
const MPU6050_Calib_t* MPU6050_GetCalibParams(void);

/* ==================== Mahony 四元数姿态解算 ==================== */

void MPU6050_QuaternionUpdate(void);         /* roll/pitch用Mahony，yaw独立积分gyro_z */
void MPU6050_QuaternionUpdateDt(float dt);   /* 使用调用方给出的真实周期(秒) */
void MPU6050_QuaternionUpdateFast(void);     /* 轻量版: 只读陀螺仪，yaw同样用梯形积分 */
void MPU6050_QuaternionUpdateFastDt(float dt);
void MPU6050_GetLatestData(MPU6050_Data_t *data); /* 取得最近一次姿态更新使用的数据 */
void MPU6050_UpdateTimeReset(void);          /* 模式切换后重置自动dt基准 */
void MPU6050_GetEuler(EulerAngles_t *euler); /* 获取欧拉角 */
void MPU6050_QuaternionReset(void);          /* Yaw归零 */

/* ==================== 互补滤波航向角 ==================== */

/******************************************************************
 * MPU6050 互补滤波航向角
 *
 * 原理：陀螺仪Z轴按真实采样周期积分提供相对 yaw。
 *       六轴 MPU6050 没有磁力计，加速度计不能修正绕重力轴的长期 yaw 漂移。
 *       alpha越大越信任陀螺仪，越小越信任加速度计
 *
 * 使用流程：
 *   1. MPU6050_Calibrate()              - 先校准零偏
 *   2. MPU6050_ComplementaryFilterUpdate() - 每个控制周期(1ms)调用
 *   3. MPU6050_GetYaw()                 - 获取航向角
 *   4. MPU6050_ResetYaw()               - 重置航向角
******************************************************************/

/**
 * @brief 更新相对航向角（Z轴陀螺仪按 Board_Millis 实际间隔积分）
 * @note  首次调用前需先 MPU6050_Calibrate；长期使用仍会有陀螺零偏漂移。
 */
void MPU6050_ComplementaryFilterUpdate(void);

/**
 * @brief 获取互补滤波后的航向角(Z轴旋转角)
 * @return 航向角(度)，顺时针为正
 */
float MPU6050_GetYaw(void);

/**
 * @brief 重置航向角为0
 */
void MPU6050_ResetYaw(void);

/**
 * @brief 设置互补滤波系数
 * @param alpha 滤波系数(0.0~1.0)，越大越信任陀螺仪，越小越信任加速度计
 *              典型值0.96~0.98，默认0.98
 */
void MPU6050_SetComplementaryAlpha(float alpha);

/* ==================== 角度PID转弯控制 ==================== */

/******************************************************************
 * MPU6050 角度PID转弯控制
 *
 * 功能：基于航向角PID控制小车转弯到指定角度
 *
 * 使用方式一（非阻塞，手动控制循环）：
 *   MPU6050_AnglePID_Init();
 *   MPU6050_AnglePID_SetTarget(90.0f);
 *   while (fabsf(MPU6050_AnglePID_GetError()) > 1.0f) {
 *       MPU6050_ComplementaryFilterUpdate();
 *       float output = MPU6050_AnglePID_Calculate();
 *       Car_SetMotors(base_speed - (int16_t)output, base_speed + (int16_t)output);
 *       delay_ms(1);
 *   }
 *
 * 使用方式二（阻塞，一步到位）：
 *   MPU6050_TurnAngle(90.0f, 200);
******************************************************************/

/**
 * @brief 初始化角度PID控制器
 * @note  默认参数: KP=5.0, KI=0.1, KD=0.5，输出范围-500~+500
 */
void MPU6050_AnglePID_Init(void);

/**
 * @brief 设置目标航向角
 * @param target_angle 目标角度(度)，相对于当前偏移
 *        正值=顺时针转，负值=逆时针转
 */
void MPU6050_AnglePID_SetTarget(float target_angle);

/**
 * @brief 执行一次角度PID计算
 * @return 角度修正量，正值表示右转修正，负值表示左转修正
 * @note  返回值可直接叠加到左右轮速度差上：
 *        left_speed  = base_speed - output
 *        right_speed = base_speed + output
 */
float MPU6050_AnglePID_Calculate(void);

/**
 * @brief 重置角度PID状态
 */
void MPU6050_AnglePID_Reset(void);

/**
 * @brief 修改角度PID参数
 * @param kp/ki/kd PID参数
 */
void MPU6050_AnglePID_SetParams(float kp, float ki, float kd);

/**
 * @brief 获取当前航向角误差
 * @return 角度误差(度)，正值表示还需右转，负值表示还需左转
 */
float MPU6050_AnglePID_GetError(void);

/**
 * @brief 精确转弯指定角度（阻塞式）
 * @param angle      转弯角度(度)，正值右转，负值左转
 * @param base_speed 转弯时的基础速度(0~999)
 * @note  阻塞式执行，直到达到目标角度后停车
 *        调用前需确保MPU6050已初始化和校准
 */
void MPU6050_TurnAngle(float angle, uint32_t base_speed);

#endif /* __MPU6050_H */
