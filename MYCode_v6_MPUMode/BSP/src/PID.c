/******************************************************************
 * 通用PID控制器模块
 *
 * 实现内容：
 *   1. PID_t 通用PID计算（含积分限幅、输出限幅、首次运行D项保护）
 *   2. 位置环 - 左右轮独立PID，输出目标速度
 *   3. 速度环 - 左右轮独立PID，内置速度采样，输出电机PWM
 *
 * 双闭环串联示例：
 *   Speed_PID_Update(left_count, right_count);
 *   Position_PID_Calculate(left_count, right_count, &l_speed, &r_speed);
 *   Speed_PID_Calculate(l_speed, r_speed, &l_pwm, &r_pwm);
 *   Car_SetMotors((int16_t)l_pwm, (int16_t)r_pwm);
******************************************************************/

#include "PID.h"
#include "ti_msp_dl_config.h"
#include "encoder.h"
#include "line.h"
#include <string.h>

/* ==================== 通用PID实现 ==================== */

/******************************************************************
 * 函 数 名 称：PID_Init
 * 函 数 说 明：初始化PID控制器，清零所有状态
 * 函 数 形 参：pid          - PID结构体指针
 *             kp/ki/kd     - PID三参数
 *             out_min      - 输出下限
 *             out_max      - 输出上限
 *             integral_max - 积分限幅(抗饱和)
 * 函 数 返 回：无
******************************************************************/
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max, float integral_max)
{
    memset(pid, 0, sizeof(PID_t));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_max = integral_max;
    pid->first_run = 1;
}

/******************************************************************
 * 函 数 名 称：PID_Calculate
 * 函 数 说 明：执行一次PID计算
 * 函 数 形 参：pid       - PID结构体指针
 *             setpoint  - 目标值
 *             feedback  - 反馈值
 * 函 数 返 回：float PID输出值
 * 备       注：位置式PID，P/I/D三项分离计算
 *             首次运行跳过微分项，避免启动尖峰
 *             包含积分限幅和输出限幅
******************************************************************/
float PID_Calculate(PID_t *pid, float setpoint, float feedback)
{
    float error = setpoint - feedback;
    float p_out, i_out, d_out, output;

    /* P项 */
    p_out = pid->kp * error;

    /* I项：积分累加 + 抗饱和限幅 */
    pid->integral += error;
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;
    i_out = pid->ki * pid->integral;

    /* D项：首次运行跳过，避免启动尖峰 */
    if (pid->first_run)
    {
        d_out = 0.0f;
        pid->first_run = 0;
    }
    else
    {
        d_out = pid->kd * (error - pid->last_error);
    }

    pid->last_error = error;

    /* 合计输出 */
    output = p_out + i_out + d_out;

    /* 输出限幅 */
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    pid->last_output = output;
    return output;
}

/******************************************************************
 * 函 数 名 称：PID_Reset
 * 函 数 说 明：重置PID控制器状态
 * 函 数 形 参：pid - PID结构体指针
 * 函 数 返 回：无
******************************************************************/
void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_output = 0.0f;
    pid->first_run = 1;
}

/******************************************************************
 * 函 数 名 称：PID_SetParams
 * 函 数 说 明：运行时修改PID参数
 * 函 数 形 参：pid - PID结构体指针, kp/ki/kd - 新参数
 * 函 数 返 回：无
******************************************************************/
void PID_SetParams(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/* ==================== 位置环 ==================== */

/* 位置环左右轮PID实例 */
static PID_t pos_pid_left;
static PID_t pos_pid_right;

/* 位置环目标值 */
static float pos_target_left = 0.0f;
static float pos_target_right = 0.0f;

/******************************************************************
 * 函 数 名 称：Position_PID_Init
 * 函 数 说 明：初始化位置环PID（默认参数，可后续修改）
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：位置环默认参数：P为主，I消除静差，D抑制超调
 *             输出范围：目标速度 -500~+500 脉冲/周期
******************************************************************/
void Position_PID_Init(void)
{
    PID_Init(&pos_pid_left,  1.0f, 0.01f, 0.5f, -500.0f, 500.0f, 1000.0f);
    PID_Init(&pos_pid_right, 1.0f, 0.01f, 0.5f, -500.0f, 500.0f, 1000.0f);
    pos_target_left = 0.0f;
    pos_target_right = 0.0f;
}

/******************************************************************
 * 函 数 名 称：Position_PID_SetTarget
 * 函 数 说 明：设置位置环目标脉冲数
 * 函 数 形 参：left_target  - 左轮目标脉冲数
 *             right_target - 右轮目标脉冲数
 * 函 数 返 回：无
 * 备       注：设定新目标时重置PID状态，避免历史积分影响
******************************************************************/
void Position_PID_SetTarget(int32_t left_target, int32_t right_target)
{
    pos_target_left = (float)left_target;
    pos_target_right = (float)right_target;
    PID_Reset(&pos_pid_left);
    PID_Reset(&pos_pid_right);
}

/******************************************************************
 * 函 数 名 称：Position_PID_Calculate
 * 函 数 说 明：执行一次位置环计算，输出左右轮目标速度
 * 函 数 形 参：left_count   - 左编码器当前计数值
 *             right_count  - 右编码器当前计数值
 *             left_speed   - 输出左轮目标速度
 *             right_speed  - 输出右轮目标速度
 * 函 数 返 回：无
******************************************************************/
void Position_PID_Calculate(int32_t left_count, int32_t right_count,
                            float *left_speed, float *right_speed)
{
    *left_speed  = PID_Calculate(&pos_pid_left,  pos_target_left,  (float)left_count);
    *right_speed = PID_Calculate(&pos_pid_right, pos_target_right, (float)right_count);
}

/******************************************************************
 * 函 数 名 称：Position_PID_Reset
 * 函 数 说 明：重置位置环状态
******************************************************************/
void Position_PID_Reset(void)
{
    PID_Reset(&pos_pid_left);
    PID_Reset(&pos_pid_right);
    pos_target_left = 0.0f;
    pos_target_right = 0.0f;
}

/******************************************************************
 * 函 数 名 称：Position_PID_SetParams
 * 函 数 说 明：修改位置环PID参数（左右轮同步修改）
******************************************************************/
void Position_PID_SetParams(float kp, float ki, float kd)
{
    PID_SetParams(&pos_pid_left,  kp, ki, kd);
    PID_SetParams(&pos_pid_right, kp, ki, kd);
}

/* ==================== 速度环 ==================== */

/* 速度环左右轮PID实例 */
static PID_t speed_pid_left;
static PID_t speed_pid_right;

/* 速度采样：上次编码器计数 */
static int32_t speed_last_left_count = 0;
static int32_t speed_last_right_count = 0;

/* 当前速度（脉冲/周期） */
static float speed_current_left = 0.0f;
static float speed_current_right = 0.0f;

/******************************************************************
 * 函 数 名 称：Speed_PID_Init
 * 函 数 说 明：初始化速度环PID（默认参数，可后续修改）
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：速度环默认参数：PI为主，D一般不用
 *             输出范围：PWM值 -999~+999
******************************************************************/
void Speed_PID_Init(void)
{
    PID_Init(&speed_pid_left,  2.0f, 0.5f, 0.0f, -999.0f, 999.0f, 500.0f);
    PID_Init(&speed_pid_right, 2.0f, 0.5f, 0.0f, -999.0f, 999.0f, 500.0f);
    speed_last_left_count = 0;
    speed_last_right_count = 0;
    speed_current_left = 0.0f;
    speed_current_right = 0.0f;
}

/******************************************************************
 * 函 数 名 称：Speed_PID_Update
 * 函 数 说 明：更新编码器速度采样（在定时中断中周期调用）
 * 函 数 形 参：left_count  - 左编码器当前计数值
 *             right_count - 右编码器当前计数值
 * 函 数 返 回：无
 * 备       注：计算两次调用间的脉冲差值作为速度
******************************************************************/
void Speed_PID_Update(int32_t left_count, int32_t right_count)
{
    speed_current_left  = (float)(left_count  - speed_last_left_count);
    speed_current_right = (float)(right_count - speed_last_right_count);

    speed_last_left_count  = left_count;
    speed_last_right_count = right_count;
}

/******************************************************************
 * 函 数 名 称：Speed_PID_Calculate
 * 函 数 说 明：执行一次速度环计算，输出左右轮PWM值
 * 函 数 形 参：left_target  - 左轮目标速度
 *             right_target - 右轮目标速度
 *             left_pwm     - 输出左轮PWM值
 *             right_pwm    - 输出右轮PWM值
 * 函 数 返 回：无
******************************************************************/
void Speed_PID_Calculate(float left_target, float right_target,
                         float *left_pwm, float *right_pwm)
{
    *left_pwm  = PID_Calculate(&speed_pid_left,  left_target,  speed_current_left);
    *right_pwm = PID_Calculate(&speed_pid_right, right_target, speed_current_right);
}

/******************************************************************
 * 函 数 名 称：Speed_PID_Reset
 * 函 数 说 明：重置速度环状态
******************************************************************/
void Speed_PID_Reset(void)
{
    PID_Reset(&speed_pid_left);
    PID_Reset(&speed_pid_right);
    speed_current_left = 0.0f;
    speed_current_right = 0.0f;
    speed_last_left_count = 0;
    speed_last_right_count = 0;
}

/******************************************************************
 * 函 数 名 称：Speed_PID_SetParams
 * 函 数 说 明：修改速度环PID参数（左右轮同步修改）
******************************************************************/
void Speed_PID_SetParams(float kp, float ki, float kd)
{
    PID_SetParams(&speed_pid_left,  kp, ki, kd);
    PID_SetParams(&speed_pid_right, kp, ki, kd);
}

/******************************************************************
 * 函 数 名 称：Speed_PID_GetCurrent
 * 函 数 说 明：获取当前实际速度
 * 函 数 形 参：left_speed  - 输出左轮速度
 *             right_speed - 输出右轮速度
 * 函 数 返 回：无
******************************************************************/
void Speed_PID_GetCurrent(float *left_speed, float *right_speed)
{
    *left_speed  = speed_current_left;
    *right_speed = speed_current_right;
}

/* ==================== 速度环目标控制与定时器ISR ==================== */

/* 速度环目标速度（编码器脉冲/周期），主循环写、ISR读 */
static volatile float s_speed_target_left = 0.0f;
static volatile float s_speed_target_right = 0.0f;

/* 速度环使能标志，ISR中检查 */
static volatile uint8_t s_speed_pid_enabled = 0;

/******************************************************************
 * 函 数 名 称：Speed_PID_SetTarget
 * 函 数 说 明：设置速度环目标速度
 * 函 数 形 参：left_target  - 左轮目标速度(编码器脉冲/周期)
 *             right_target - 右轮目标速度(编码器脉冲/周期)
 * 函 数 返 回：无
 * 备       注：由主循环调用，ISR中读取执行PID计算
******************************************************************/
void Speed_PID_SetTarget(float left_target, float right_target)
{
    s_speed_target_left = left_target;
    s_speed_target_right = right_target;
}

/******************************************************************
 * 函 数 名 称：Speed_PID_Enable
 * 函 数 说 明：使能速度环
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：重置PID状态，清零目标，ISR开始输出电机PWM
******************************************************************/
void Speed_PID_Enable(void)
{
    s_speed_pid_enabled = 1;
    Speed_PID_Reset();
    s_speed_target_left = 0.0f;
    s_speed_target_right = 0.0f;
}

/******************************************************************
 * 函 数 名 称：Speed_PID_Disable
 * 函 数 说 明：禁止速度环并刹车
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：清零目标，ISR停止输出，调用Car_Stop()刹车
******************************************************************/
void Speed_PID_Disable(void)
{
    s_speed_pid_enabled = 0;
    s_speed_target_left = 0.0f;
    s_speed_target_right = 0.0f;
    Car_Stop();
}

/******************************************************************
 * 函 数 名 称：Speed_PID_IsEnabled
 * 函 数 说 明：查询速度环是否使能
 * 函 数 返 回：1-使能 0-禁止
******************************************************************/
uint8_t Speed_PID_IsEnabled(void)
{
    return s_speed_pid_enabled;
}

/******************************************************************
 * 函 数 名 称：Speed_PID_Timer_Init
 * 函 数 说 明：初始化速度环定时器并启动
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：使能TIMG0 NVIC中断（优先级低于编码器），启动定时器
 *             定时器周期由SysConfig配置为5ms
******************************************************************/
void Speed_PID_Timer_Init(void)
{
    /* 编码器ISR优先级=0(最高)，速度环优先级=1，确保编码器计数不被打断 */
    NVIC_SetPriority(PID_TIMER_INST_INT_IRQN, 1);
    NVIC_EnableIRQ(PID_TIMER_INST_INT_IRQN);
    DL_Timer_startCounter(PID_TIMER_INST);
}

/******************************************************************
 * 函 数 名 称：PID_TIMER_INST_IRQHandler
 * 函 数 说 明：速度环定时器中断服务函数（5ms周期）
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：速度环使能时自动执行：
 *             1. 读取编码器累计计数
 *             2. Speed_PID_Update()  计算当前速度(脉冲/周期)
 *             3. Speed_PID_Calculate() PID计算输出PWM
 *             4. Car_SetMotors()     输出到电机
******************************************************************/
void PID_TIMER_INST_IRQHandler(void)
{
    if (DL_TimerG_getRawInterruptStatus(PID_TIMER_INST,
                                     DL_TIMERG_INTERRUPT_ZERO_EVENT))
    {
        DL_TimerG_clearInterruptStatus(PID_TIMER_INST,
                                       DL_TIMERG_INTERRUPT_ZERO_EVENT);

        if (s_speed_pid_enabled)
        {
            /* 读取编码器累计计数 */
            int32_t left_count  = Encoder_GetLeft();
            int32_t right_count = Encoder_GetRight();

            /* 计算当前速度（两次调用间的脉冲差值） */
            Speed_PID_Update(left_count, right_count);

            /* PID计算：目标速度 → PWM输出 */
            float left_pwm, right_pwm;
            float target_l = s_speed_target_left;
            float target_r = s_speed_target_right;
            Speed_PID_Calculate(target_l, target_r, &left_pwm, &right_pwm);

            /* 输出到电机 */
            Car_SetMotors((int16_t)left_pwm, (int16_t)right_pwm);
        }
    }
}
