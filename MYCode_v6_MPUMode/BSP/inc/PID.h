#ifndef _PID_H
#define _PID_H

#include "board.h"

/******************************************************************
 * 通用PID控制器模块
 *
 * 提供：
 *   1. PID_t 通用PID结构体与计算接口，可复用于位置环/速度环/角度环
 *   2. 位置环 - 左右轮独立PID，基于编码器脉冲控制行走距离
 *   3. 速度环 - 左右轮独立PID，基于编码器脉冲速率控制电机转速
 *
 * 双闭环串联示例：
 *   Speed_PID_Update(left_count, right_count);
 *   Position_PID_Calculate(left_count, right_count, &l_speed, &r_speed);
 *   Speed_PID_Calculate(l_speed, r_speed, &l_pwm, &r_pwm);
 *   Car_SetMotors((int16_t)l_pwm, (int16_t)r_pwm);
******************************************************************/

/* ==================== 通用PID结构体 ==================== */

typedef struct {
    /* PID参数 */
    float kp;               /* 比例增益 */
    float ki;               /* 积分增益 */
    float kd;               /* 微分增益 */

    /* 限幅 */
    float out_min;          /* 输出下限 */
    float out_max;          /* 输出上限 */
    float integral_max;     /* 积分限幅(抗饱和) */

    /* 内部状态 */
    float integral;         /* 积分累计 */
    float last_error;       /* 上一次误差 */
    float last_output;      /* 上一次输出 */

    /* 标志 */
    uint8_t first_run;      /* 首次运行标志，跳过微分项避免启动尖峰 */
} PID_t;

/******************************************************************
 * 函 数 名 称：PID_Init
 * 函 数 说 明：初始化PID控制器
 * 函 数 形 参：pid    - PID结构体指针
 *             kp/ki/kd - PID参数
 *             out_min/out_max - 输出限幅范围
 *             integral_max   - 积分限幅值
 * 函 数 返 回：无
******************************************************************/
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max, float integral_max);

/******************************************************************
 * 函 数 名 称：PID_Calculate
 * 函 数 说 明：执行一次PID计算
 * 函 数 形 参：pid      - PID结构体指针
 *             setpoint - 目标值
 *             feedback - 反馈值(实际值)
 * 函 数 返 回：float PID输出值
 * 备       注：首次运行跳过微分项，避免启动尖峰
******************************************************************/
float PID_Calculate(PID_t *pid, float setpoint, float feedback);

/******************************************************************
 * 函 数 名 称：PID_Reset
 * 函 数 说 明：重置PID内部状态(积分清零、重置首次运行标志)
 * 函 数 形 参：pid - PID结构体指针
 * 函 数 返 回：无
******************************************************************/
void PID_Reset(PID_t *pid);

/******************************************************************
 * 函 数 名 称：PID_SetParams
 * 函 数 说 明：运行时修改PID参数
 * 函 数 形 参：pid - PID结构体指针, kp/ki/kd - 新参数
 * 函 数 返 回：无
******************************************************************/
void PID_SetParams(PID_t *pid, float kp, float ki, float kd);

/* ==================== 位置环 ==================== */

/******************************************************************
 * 位置环 - 左右轮独立PID，基于编码器累计脉冲控制行走距离
 * 典型用法：设定目标脉冲数，位置环输出目标速度给速度环
******************************************************************/

void Position_PID_Init(void);
void Position_PID_SetTarget(int32_t left_target, int32_t right_target);
void Position_PID_Calculate(int32_t left_count, int32_t right_count,
                            float *left_speed, float *right_speed);
void Position_PID_Reset(void);
void Position_PID_SetParams(float kp, float ki, float kd);

/* ==================== 速度环 ==================== */

/******************************************************************
 * 速度环 - 左右轮独立PID，基于编码器脉冲速率控制电机转速
 * 典型用法：接收位置环输出的目标速度，速度环输出电机PWM值
 *
 * 使用流程：
 *   1. Speed_PID_Init()       - 初始化速度环PID参数
 *   2. Speed_PID_Timer_Init() - 初始化PID定时器(5ms周期)并启动
 *   3. Speed_PID_Enable()     - 使能速度环，定时器ISR自动运行
 *   4. Speed_PID_SetTarget()  - 设置目标速度(编码器脉冲/周期)
 *   5. Speed_PID_Disable()    - 停止速度环并刹车
 *
 * 定时器ISR自动执行：
 *   读编码器 → Speed_PID_Update() → Speed_PID_Calculate() → Car_SetMotors()
 *
 * PWM目标值到编码器脉冲/周期的转换：
 *   target_enc = pwm_value * SPEED_PID_ENCODER_SCALE
 *   SCALE需根据实际硬件标定：
 *     1. 给定PWM值(如130)，测量5ms内编码器脉冲数(如50)
 *     2. SCALE = 50 / 130 ≈ 0.38
******************************************************************/

/* PWM单位到编码器脉冲/周期的缩放因子，需根据实际硬件标定 */
#define SPEED_PID_ENCODER_SCALE  0.4f

void Speed_PID_Init(void);
void Speed_PID_Update(int32_t left_count, int32_t right_count);
void Speed_PID_Calculate(float left_target, float right_target,
                         float *left_pwm, float *right_pwm);
void Speed_PID_Reset(void);
void Speed_PID_SetParams(float kp, float ki, float kd);
void Speed_PID_GetCurrent(float *left_speed, float *right_speed);

/******************************************************************
 * 速度环目标控制（主循环调用，ISR读取）
******************************************************************/
void Speed_PID_SetTarget(float left_target, float right_target);

/******************************************************************
 * 速度环使能/禁止
 * Enable:  重置PID状态，清零目标，ISR开始输出电机PWM
 * Disable: 清零目标，ISR停止输出，调用Car_Stop()刹车
******************************************************************/
void Speed_PID_Enable(void);
void Speed_PID_Disable(void);
uint8_t Speed_PID_IsEnabled(void);

/******************************************************************
 * 速度环定时器初始化
 * 使能NVIC(优先级低于编码器)，启动TIMG0定时器
******************************************************************/
void Speed_PID_Timer_Init(void);

#endif /* _PID_H */
