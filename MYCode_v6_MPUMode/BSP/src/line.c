/******************************************************************
 * 巡线模块 - 八路灰度传感器 PD 差速巡线 + MPU6050 航向角 PID 辅助
 *
 * 算法原理（参考电赛/智能车竞赛主流方案）：
 *   1. 八路灰度传感器检测黑线位置，加权求均值得到位置偏差 error
 *      error < 0 表示线偏左，error > 0 表示线偏右，error = 0 居中
 *   2. 位置偏差经 EMA 低通滤波 + 中心死区处理后，送入 PD 控制器
 *      line_pd = KP * filtered_error + KD * (filtered_error - last_error)
 *   3. 【MPU 辅助层 - 航向角 PID（外环）】
 *      a. 直行稳航：居中 20ms 后锁定当前 yaw，用 PID 维持航向
 *         yaw_pid = YAWP * yaw_err + YAWI * ∫yaw_err + YAWD * Δyaw_err
 *         pid_output = 0.6*line_pd + 0.4*yaw_pid   (直行时航向权重高)
 *      b. 转弯阻尼：用陀螺仪 Z 轴角速度做连续阻尼，抑制甩尾
 *         pid_output = line_pd - YAW_GYRO_K * gyro_z  (连续函数，不分档)
 *      c. 丢线航向保持：锁住丢线瞬间 yaw，用同一套 yaw PID 维持航向
 *         叠加小幅摆动搜索（sine 波），帮车找回黑线
 *   4. PD 输出作为左右轮速度差，实现连续平滑的差速转弯
 *      left_speed  = BASE_SPEED - pid_output
 *      right_speed = BASE_SPEED + pid_output
 *
 * 传感器排列与权重：
 *   LEFT1  LEFT2  LEFT3  LEFT4  RIGHT1  RIGHT2  RIGHT3  RIGHT4
 *   -4     -3     -2     -1     1       2       3       4
 *
 * 可调参数（通过 getter/setter 运行时修改）：
 *   g_line_kp         - 比例增益，越大转弯越灵敏，过大会震荡
 *   g_line_kd         - 微分增益，越大阻尼越强，过大会迟钝
 *   g_line_lost_gain  - 丢线修正增益（非MPU模式），越大丢线后找线越猛
 *   g_line_base_speed - 基础直行速度，左右轮速度的基准值
 *
 * MPU 固定参数（compile-time，参考网上主流值）：
 *   YAW_KP       - 航向角 P 增益，1°误差→此值速度差 (推荐 3.0~6.0)
 *   YAW_KI       - 航向角 I 增益，消除电机不对称造成的缓慢偏移
 *   YAW_KD       - 航向角 D 增益，抑制航向修正过冲
 *   YAW_GYRO_K   - 转弯陀螺阻尼系数，越大转弯越稳但可能迟钝
 *   YAW_LOCK_MS  - 居中多少毫秒后锁定航向 (推荐 15~30)
 *
 * 一般为固定参数：
 *   LINE_DEAD_ZONE    - 中心死区，误差绝对值 <= 此值时视为居中不修正
 *   LINE_FILTER_SHIFT - EMA滤波移位位数，1=50%旧+50%新，2=75%旧+25%新
 ******************************************************************/

#include "line.h"
#include "bsp_tb6612.h"
#include "mpu6050.h"
#include "ti_msp_dl_config.h"
#include <math.h>

/* ==================== 可调参数（运行时可通过 setter 修改） ==================== */

/* 比例增益：位置偏差 → 速度差的放大倍数，值越大转弯越灵敏 */
static int16_t g_line_kp = 20;

/* 微分增益：偏差变化率 → 速度差的放大倍数，抑制振荡让转弯更柔和 */
static int16_t g_line_kd = 35;

/* 丢线修正增益：丢线时用此增益替代 KP，值越大丢线后找线越猛 */
static int16_t g_line_lost_gain = 23;

/* 基础直行速度：左右轮速度的基准值(0~999)，PD输出在此基础上加减 */
static int16_t g_line_base_speed = 130;

/* 直角弯 pivot 最小保持时间(ms)，在此期间不检查退出条件 */
static int16_t g_line_pivot_time = 100;

/* 直角弯外侧轮速度：由菜单 TurnSpd 调整并保存到 Flash。 */
static int16_t g_line_turn_speed = 200;

/* MPU6050 辅助模式：1=启用, 0=关闭 */
static uint8_t g_mpu_mode = 0;

/* ==================== 固定参数 ==================== */

/* 中心死区：原始误差绝对值 <= 此值时视为居中，不做修正（消除中心微震荡） */
#define LINE_DEAD_ZONE     1

/* EMA 滤波移位位数：filtered = (old + new) >> N
 * N=1 → 50%旧+50%新（响应快，滤波弱）
 * N=2 → 75%旧+25%新（响应慢，滤波强） */
#define LINE_FILTER_SHIFT  1

/* 最小前进速度：转弯时内侧轮不低于此值，防止原地打转
 * 值太小→内侧轮停转→车原地转；值太大→差速不够→转弯半径大 */
#define MIN_FORWARD_SPEED  55

/* 直角弯 pivot 速度：检测到最外侧传感器时，内外轮以此速度差速旋转 */
/* 直角入口连续确认时间：滤掉最外侧传感器的单拍毛刺，同时保持快速响应。 */
#define PIVOT_ENTRY_CONFIRM_MS  2U

/* 新线路必须由中心两路连续确认，避免刚擦到线路边缘就过早退出 pivot。 */
#define PIVOT_EXIT_CONFIRM_MS   5U
#define PIVOT_CENTER_MASK       0x18U   /* LEFT4 + RIGHT1，传感器阵列中央两路 */

/* pivot 超时只用于防止传感器异常时无限旋转，正常退出仍由中心重捕获决定。 */
#define PIVOT_TIMEOUT_MS        900U

/* 内侧轮反转力度。直角弯使用小幅反转形成原地旋转力矩，比锁死内轮可靠。 */
#define PIVOT_REVERSE_MIN       65
#define PIVOT_REVERSE_MAX       140

/* ==================== 内部状态变量 ==================== */

/* 上一拍滤波后误差，用于 PD 的微分项计算 */
static int16_t pid_last_error = 0;

/* 最后一次有效（非丢线）的原始误差，丢线时沿用此方向 */
static int16_t pid_last_valid_error = 0;

/* 是否至少读到过一次有效黑线；上电未见线时禁止默认向右搜索。 */
static uint8_t line_has_last = 0;

/* 最近一次明确偏左/偏右的方向，用于全黑交叉图案下消除左右歧义。 */
static int8_t last_nonzero_dir = 0;

/* EMA 滤波后的误差值，PD 控制器的实际输入 */
static int16_t filtered_error = 0;

/* 直角弯 pivot 状态：检测到 LEFT1 或 RIGHT4 后进入，直到中心传感器重新触发 */
static uint8_t  pivot_active = 0;   /* 0=正常巡线, 1=直角弯pivot中 */
static int16_t  pivot_dir = 0;      /* +1=右转, -1=左转 */
static uint16_t pivot_timer = 0;    /* 最小保持时间(ms) */
static uint8_t  block_opposite = 0; /* 退出后封锁反向入口(ms) */
static int8_t   pivot_candidate_dir = 0; /* 入口去抖期间记录候选方向 */
static uint8_t  pivot_candidate_cnt = 0; /* 候选方向连续出现次数 */
static uint8_t  pivot_departed = 0;      /* 已离开触发 pivot 的旧线路外侧 */
static uint8_t  pivot_center_cnt = 0;    /* 中心两路稳定重捕获计数 */

/* ==================== MPU 航向角 PID 参数（参考网上主流值） ==================== */

/* 航向角 P 增益：1°误差 = YAWP 的速度差。参考范围 3.0~6.0，直道取低、弯道取高 */
#define YAW_KP       3.0f

/* 直接用实测角速度作为阻尼（测量微分），不依赖不稳定的循环次数差分。 */
#define YAW_RATE_K   0.25f

/* 转弯陀螺阻尼系数：gyro_z(°/s) 直接衰减 PD 输出，连续函数不分档
 * 参考范围 0.2~0.6，值越大转弯越稳但可能迟钝 */
#define YAW_GYRO_K   0.35f

/* 居中多少毫秒后锁定航向。参考范围 15~30ms */
#define YAW_LOCK_MS  20

/* 丢线后搜索摆幅（速度差），小幅左右摆动帮车找回黑线 */
#define LOST_SEARCH_AMPLITUDE  15

/* ==================== MPU 航向角 PID 内部状态 ==================== */

/* 航向角 PID 积分项（I 累积） */
static float   yaw_i = 0;

/* 航向角 PID 上一次误差（D 用） */
static float   yaw_last_err = 0;

/* 锁定的目标航向角 */
static float   yaw_target = 0;

/* 航向锁定标志：1=已锁定，丢线/直行时生效 */
static uint8_t  yaw_locked = 0;

/* 居中连续计数：超过 YAW_LOCK_MS 后锁定航向 */
static uint8_t  center_cnt = 0;

/* 丢线后搜索相位（正弦波相位累加，产生小幅摆动） */
static uint16_t lost_search_phase = 0;

/* 传感器权重表：8路传感器从左到右，负值偏左，正值偏右，均匀分布 */
static const int8_t sensor_weights[LINE_SENSOR_COUNT] = {-4, -3, -2, -1, 1, 2, 3, 4};

/******************************************************************
 * 函 数 名 称：Car_Forward
 * 函 数 说 明：小车前进（左轮反转+右轮正转 = 前进方向）
 * 函 数 形 参：speedL - 左电机速度(0~999)  speedR - 右电机速度(0~999)
 * 函 数 返 回：无
******************************************************************/
void Car_Forward(uint32_t speedL, uint32_t speedR)
{
    AO_Control(0, speedL);
    BO_Control(1, speedR);
}

/******************************************************************
 * 函 数 名 称：Car_TurnLeft
 * 函 数 说 明：小车原地左转（左轮反转+右轮反转）
 * 函 数 形 参：speedL - 左电机速度(0~999)  speedR - 右电机速度(0~999)
 * 函 数 返 回：无
******************************************************************/
void Car_TurnLeft(uint32_t speedL, uint32_t speedR)
{
    AO_Control(0, speedL);
    BO_Control(0, speedR);
}

/******************************************************************
 * 函 数 名 称：Car_TurnRight
 * 函 数 说 明：小车原地右转（左轮正转+右轮正转）
 * 函 数 形 参：speedL - 左电机速度(0~999)  speedR - 右电机速度(0~999)
 * 函 数 返 回：无
******************************************************************/
void Car_TurnRight(uint32_t speedL, uint32_t speedR)
{
    AO_Control(1, speedL);
    BO_Control(1, speedR);
}

/******************************************************************
 * 函 数 名 称：Car_Stop
 * 函 数 说 明：小车停止（TB6612刹车模式，两输入同高）
 * 函 数 形 参：无
 * 函 数 返 回：无
******************************************************************/
void Car_Stop(void)
{
    TB6612_Motor_Stop();
}

/******************************************************************
 * 函 数 名 称：Car_SetMotors
 * 函 数 说 明：带符号的双电机速度控制，正值前进，负值后退
 * 函 数 形 参：speedL - 左电机速度(-999~+999)  speedR - 右电机速度(-999~+999)
 * 函 数 返 回：无
 * 备       注：PD控制器输出正负值，此函数自动映射到电机正反转
******************************************************************/
void Car_SetMotors(int16_t speedL, int16_t speedR)
{
    /* 左电机：正值反转(前进)，负值正转(后退) */
    if (speedL >= 0)
        AO_Control(0, (uint32_t)speedL);
    else
        AO_Control(1, (uint32_t)(-speedL));

    /* 右电机：正值正转(前进)，负值反转(后退) */
    if (speedR >= 0)
        BO_Control(1, (uint32_t)speedR);
    else
        BO_Control(0, (uint32_t)(-speedR));
}

/******************************************************************
 * 函 数 名 称：Line_Tracking_Init
 * 函 数 说 明：巡线模块初始化，停车并重置所有PD状态变量
 * 函 数 形 参：无
 * 函 数 返 回：无
******************************************************************/
void Line_Tracking_Init(void)
{
    Car_Stop();
    pid_last_error = 0;
    pid_last_valid_error = 0;
    filtered_error = 0;
    line_has_last = 0;
    last_nonzero_dir = 0;
    pivot_active = 0;
    pivot_dir = 0;
    pivot_timer = 0;
    block_opposite = 0;
    pivot_candidate_dir = 0;
    pivot_candidate_cnt = 0;
    pivot_departed = 0;
    pivot_center_cnt = 0;
    yaw_locked = 0;
    center_cnt = 0;
    yaw_i = 0.0f;
    yaw_last_err = 0.0f;
}

/******************************************************************
 * 函 数 名 称：Line_ReadSensors
 * 函 数 说 明：读取八路灰度传感器状态
 * 函 数 形 参：无
 * 函 数 返 回：uint8_t 8位掩码，bit0=LEFT1 ~ bit7=RIGHT4，1=检测到黑线
******************************************************************/
uint8_t Line_ReadSensors(void)
{
    uint8_t sensors = 0;

    /* 逐位读取传感器，检测到黑线（高电平）置1 */
    if (DL_GPIO_readPins(LINE_LEFT1_PORT, LINE_LEFT1_PIN) > 0) sensors |= 0x01;
    if (DL_GPIO_readPins(LINE_LEFT2_PORT, LINE_LEFT2_PIN) > 0) sensors |= 0x02;
    if (DL_GPIO_readPins(LINE_LEFT3_PORT, LINE_LEFT3_PIN) > 0) sensors |= 0x04;
    if (DL_GPIO_readPins(LINE_LEFT4_PORT, LINE_LEFT4_PIN) > 0) sensors |= 0x08;
    if (DL_GPIO_readPins(LINE_RIGHT1_PORT, LINE_RIGHT1_PIN) > 0) sensors |= 0x10;
    if (DL_GPIO_readPins(LINE_RIGHT2_PORT, LINE_RIGHT2_PIN) > 0) sensors |= 0x20;
    if (DL_GPIO_readPins(LINE_RIGHT3_PORT, LINE_RIGHT3_PIN) > 0) sensors |= 0x40;
    if (DL_GPIO_readPins(LINE_RIGHT4_PORT, LINE_RIGHT4_PIN) > 0) sensors |= 0x80;

    return sensors;
}

/******************************************************************
 * 函 数 名 称：Line_GetPosition
 * 函 数 说 明：根据传感器状态计算加权位置值
 * 函 数 形 参：sensors - 传感器掩码
 * 函 数 返 回：int16_t 位置值，负值偏左，正值偏右，0居中
 * 备       注：对触发的传感器权重求均值，值域约 [-4, +4]
 *             例：仅LEFT4(-1)触发 → 返回-1，仅RIGHT2(+2)触发 → 返回+2
******************************************************************/
int16_t Line_GetPosition(uint8_t sensors)
{
    int16_t weighted_sum = 0;
    uint8_t count = 0;

    /* 对每个检测到黑线的传感器累加权重 */
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        if (sensors & (1 << i))
        {
            weighted_sum += sensor_weights[i];
            count++;
        }
    }

    /* 无传感器触发时返回0 */
    if (count == 0)
        return 0;

    return weighted_sum / count;
}

/******************************************************************
 * 函 数 名 称：Line_GetState
 * 函 数 说 明：根据传感器状态判断小车运动方向（分档方式，备用接口）
 * 函 数 形 参：无
 * 函 数 返 回：CarState_t 运动状态枚举
 * 备       注：当前主流程使用PD连续控制，此函数保留用于兼容
******************************************************************/
CarState_t Line_GetState(void)
{
    uint8_t sensors = Line_ReadSensors();

    /* 所有传感器都未触发=丢线 */
    if (sensors == 0x00)
    {
        return DIR_LOST;
    }

    int16_t position = Line_GetPosition(sensors);

    /* 根据加权位置值判断方向 */
    if (position >= -1 && position <= 1)
    {
        return DIR_FORWARD;
    }
    else if (position < -1 && position >= -2)
    {
        return DIR_LEFT_SLIGHT;
    }
    else if (position < -2 && position >= -3)
    {
        return DIR_LEFT_MEDIUM;
    }
    else if (position < -3)
    {
        return DIR_LEFT_SHARP;
    }
    else if (position > 1 && position <= 2)
    {
        return DIR_RIGHT_SLIGHT;
    }
    else if (position > 2 && position <= 3)
    {
        return DIR_RIGHT_MEDIUM;
    }
    else
    {
        return DIR_RIGHT_SHARP;
    }
}

/******************************************************************
 * 函 数 名 称：Line_Execute
 * 函 数 说 明：根据运动状态执行对应的电机控制（分档方式，备用接口）
 * 函 数 形 参：state - 运动状态
 * 函 数 返 回：无
 * 备       注：当前主流程使用PD连续控制，此函数保留用于兼容
******************************************************************/
void Line_Execute(CarState_t state)
{
    switch (state)
    {
        case DIR_FORWARD:
            Car_Forward(150, 150);
            break;
        case DIR_LEFT_SLIGHT:
            Car_Forward(150, 125);
            break;
        case DIR_LEFT_MEDIUM:
            Car_Forward(150, 110);
            break;
        case DIR_LEFT_SHARP:
            Car_TurnLeft(200, 200);
            break;
        case DIR_RIGHT_SLIGHT:
            Car_Forward(125, 150);
            break;
        case DIR_RIGHT_MEDIUM:
            Car_Forward(110, 150);
            break;
        case DIR_RIGHT_SHARP:
            Car_TurnRight(200, 200);
            break;
        default:
            Car_Forward(120, 120);
            break;
    }
}

/******************************************************************
 * 函 数 名 称：yaw_error_wrap
 * 函 数 说 明：计算航向角误差并做 ±180° 跨越处理
 * 函 数 形 参：target - 目标航向角(度)  current - 当前航向角(度)
 * 函 数 返 回：float 规范化后的角度误差 (current - target)，范围 -180~+180
 * 备       注：正值需右转修正，负值需左转修正
 ******************************************************************/
static float yaw_error_wrap(float target, float current)
{
    float err = current - target;
    while (err > 180.0f)  err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

/******************************************************************
 * 函 数 名 称：yaw_pid_compute
 * 函 数 说 明：计算航向角 PD 输出（角度 P + 实测角速度阻尼）
 * 函 数 形 参：yaw_err - 规范化后的航向角误差(度)，gyro_z - 实测角速度
 * 函 数 返 回：int16_t 航向角 PID 输出值（速度差），正值=右转修正
 * 备       注：参考网上主流方案，Yaw PID 作为外环输出速度差
 *             直接叠加到巡线 PD 输出上：
 *             left_speed  = base - (line_pd + yaw_pid)
 *             right_speed = base + (line_pd + yaw_pid)
 ******************************************************************/
static int16_t yaw_pid_compute(float yaw_err, float gyro_z)
{
    /* 实测为逆时针 yaw/gyro_z 为正，而 pid_output 正值驱动车体顺时针修正。
     * 因此使用 current-target，并让角速度项与误差项同号形成阻尼。 */
    float total = YAW_KP * yaw_err + YAW_RATE_K * gyro_z;
    yaw_i = 0.0f;
    yaw_last_err = yaw_err;

    /* 输出限幅 */
    if (total > 300.0f)  total = 300.0f;
    if (total < -300.0f) total = -300.0f;
    return (int16_t)total;
}

/******************************************************************
 * 函 数 名 称：Line_Run
 * 函 数 说 明：巡线主循环单次执行，PD + MPU 航向角 PID 串级控制
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：处理流程（参考电赛/智能车竞赛主流方案）：
 *             1. 读取传感器 → 2. 更新 MPU 姿态(yaw+gyro_z)
 *             → 3. pivot 判断 → 4. 丢线/有线分支
 *             → 4a. 有线：位置偏差 → 死区 → EMA 滤波 → 巡线 PD
 *                  ├ 居中 → 锁定 yaw → 航向角 PID 稳航 → 加权融合(60%PD+40%YawPID)
 *                  └ 转弯 → 释放 yaw 锁 → 陀螺角速度连续阻尼 → 90%PD+10%陀螺阻尼
 *             → 4b. 丢线(MPU)：锁定 yaw → 航向角 PID 保持 + 正弦搜索摆动
 *             → 4c. 丢线(非MPU)：沿用最后有效误差方向
 *             → 5. 速度限幅 → 6. 电机输出
 *
 *             PD 始终是主控（内环），Yaw PID 是辅助（外环）
 *             巡线 PD 决定"怎么跟着线走"
 *             航向角 PID 保证"走的时候车身不歪"
 *             陀螺阻尼防止"转弯时甩过头"
 ******************************************************************/
void Line_Run(void)
{
    uint8_t sensors = Line_ReadSensors();

    /* ---- MPU6050 数据更新 ---- */
    float mpu_gyro_z = 0;
    float mpu_yaw = 0;
    if (g_mpu_mode)
    {
        MPU6050_QuaternionUpdate();
        MPU6050_Data_t mpu_data;
        MPU6050_GetLatestData(&mpu_data);
        mpu_gyro_z = mpu_data.gyro_z;

        EulerAngles_t euler;
        MPU6050_GetEuler(&euler);
        mpu_yaw = euler.yaw;
    }

    /* ========== 直角弯 pivot 状态机 ========== */
    if (block_opposite > 0) block_opposite--;

    if (!pivot_active && block_opposite == 0)
    {
        int8_t detected_dir = 0;
        uint8_t left_outer  = (uint8_t)(sensors & 0x01U);
        uint8_t right_outer = (uint8_t)(sensors & 0x80U);

        if (left_outer && !right_outer)
        {
            detected_dir = -1;
        }
        else if (right_outer && !left_outer)
        {
            detected_dir = 1;
        }
        else if (left_outer && right_outer)
        {
            /* 横线或粗线可能让两端同时触发。先比较左右半区数量，再使用
             * 最近一次明确转向，避免原代码固定优先左转。 */
            uint8_t left_count = 0;
            uint8_t right_count = 0;
            for (uint8_t i = 0; i < 4U; i++)
            {
                if (sensors & (1U << i)) left_count++;
                if (sensors & (1U << (i + 4U))) right_count++;
            }

            if (left_count > right_count) detected_dir = -1;
            else if (right_count > left_count) detected_dir = 1;
            else if (pivot_candidate_dir != 0) detected_dir = pivot_candidate_dir;
            else detected_dir = last_nonzero_dir;
        }

        /* 最外侧信号需要连续出现，单拍噪声不会直接触发强制旋转。 */
        if (detected_dir != 0)
        {
            if (pivot_candidate_dir == detected_dir)
            {
                if (pivot_candidate_cnt < PIVOT_ENTRY_CONFIRM_MS)
                    pivot_candidate_cnt++;
            }
            else
            {
                pivot_candidate_dir = detected_dir;
                pivot_candidate_cnt = 1;
            }

            if (pivot_candidate_cnt >= PIVOT_ENTRY_CONFIRM_MS)
            {
                pivot_active = 1;
                pivot_dir = detected_dir;
                pivot_timer = 0;
                pivot_departed = 0;
                pivot_center_cnt = 0;
                pivot_candidate_dir = 0;
                pivot_candidate_cnt = 0;

                /* pivot 期间不使用旧的直行航向锁，防止 MPU 辅助与转弯打架。 */
                yaw_locked = 0;
                center_cnt = 0;
                yaw_i = 0;
                yaw_last_err = 0;
            }
        }
        else
        {
            pivot_candidate_dir = 0;
            pivot_candidate_cnt = 0;
        }
    }

    if (pivot_active)
    {
        if (pivot_timer < 0xFFFFU) pivot_timer++;

        /* 原触发侧的最外路释放后，说明车身已经离开进入直角前的旧线路。 */
        if ((pivot_dir < 0 && (sensors & 0x01U) == 0U) ||
            (pivot_dir > 0 && (sensors & 0x80U) == 0U))
        {
            pivot_departed = 1;
        }

        int16_t fast = g_line_turn_speed;
        if (fast < 80) fast = 80;
        if (fast > 250) fast = 250;
        int16_t reverse = fast / 2;
        if (reverse < PIVOT_REVERSE_MIN) reverse = PIVOT_REVERSE_MIN;
        if (reverse > PIVOT_REVERSE_MAX) reverse = PIVOT_REVERSE_MAX;

        /* Car_SetMotors 的第一个通道对应车体右轮、第二个对应左轮。
         * 右转：右轮小幅后退、左轮前进；左转反之。 */
        if (pivot_dir > 0) Car_SetMotors(-reverse, fast);
        else               Car_SetMotors(fast, -reverse);

        /* 必须满足三个条件才退出：已经离开旧线、达到菜单设定的最小旋转
         * 时间、中央两路连续稳定检测到新线。 */
        if (pivot_departed &&
            pivot_timer >= (uint16_t)g_line_pivot_time &&
            (sensors & PIVOT_CENTER_MASK) != 0U)
        {
            if (pivot_center_cnt < PIVOT_EXIT_CONFIRM_MS) pivot_center_cnt++;
        }
        else
        {
            pivot_center_cnt = 0;
        }

        if (pivot_center_cnt >= PIVOT_EXIT_CONFIRM_MS ||
            pivot_timer >= PIVOT_TIMEOUT_MS)
        {
            int16_t completed_dir = pivot_dir;
            pivot_active = 0;
            pivot_dir = 0;
            pivot_timer = 0;
            pivot_departed = 0;
            pivot_center_cnt = 0;
            block_opposite = 50;

            /* 清除 PD 瞬态，但保留刚完成的转向方向。若新线短暂再次丢失，
             * 搜索会继续沿正确方向，而不是像原代码一样固定向右。 */
            filtered_error = 0;
            pid_last_error = 0;
            pid_last_valid_error = completed_dir * 3;
            last_nonzero_dir = (int8_t)completed_dir;
            line_has_last = 1;
        }
        return;
    }

    /* ========== PD + MPU 串级控制 ========== */
    int16_t raw_error;
    int16_t pid_output;

    if (sensors == 0x00)
    {
        /* ================================================================ */
        /*  丢线处理                                                         */
        /* ================================================================ */
        if (g_mpu_mode)
        {
            /* ---- MPU 模式：航向角 PID 保持 + 正弦搜索 ---- */
            if (!yaw_locked)
            {
                /* 丢线第一拍：锁定当前 yaw + 重置航向 PID 状态 */
                yaw_target = mpu_yaw;
                yaw_locked = 1;
                yaw_i = 0;
                yaw_last_err = 0;
                lost_search_phase = 0;
            }

            /* 航向角 PID：维持丢线前的方向 */
            float yaw_err = yaw_error_wrap(yaw_target, mpu_yaw);
            int16_t yaw_out = yaw_pid_compute(yaw_err, mpu_gyro_z);

            /* 叠加正弦搜索摆动：帮助车在横向找回黑线
             * 摆幅 = LOST_SEARCH_AMPLITUDE，周期约 200ms */
            lost_search_phase++;
            float search_sweep = (float)LOST_SEARCH_AMPLITUDE
                * sinf((float)lost_search_phase * 0.0314f); /* 2π/200 ≈ 0.0314 */

            pid_output = yaw_out + (int16_t)search_sweep;

            /* 限幅 */
            if (pid_output > 280) pid_output = 280;
            if (pid_output < -280) pid_output = -280;

            filtered_error = 0;
        }
        else
        {
            /* 非 MPU 模式：沿用最后有效误差方向 */
            if (!line_has_last)
            {
                /* 上电后尚未见到黑线时缓慢直行，不再默认向右急搜。 */
                pid_output = 0;
                filtered_error = 0;
            }
            else
            {
                raw_error = pid_last_valid_error;
                if (raw_error >= -LINE_DEAD_ZONE && raw_error <= LINE_DEAD_ZONE)
                    raw_error = (last_nonzero_dir >= 0) ? 3 : -3;
                pid_output = g_line_lost_gain * raw_error;
                filtered_error = raw_error;
            }
        }
    }
    else
    {
        /* ================================================================ */
        /*  有线：巡线 PD + MPU 辅助                                         */
        /* ================================================================ */

        raw_error = Line_GetPosition(sensors);
        pid_last_valid_error = raw_error;
        line_has_last = 1;
        if (raw_error < -LINE_DEAD_ZONE) last_nonzero_dir = -1;
        else if (raw_error > LINE_DEAD_ZONE) last_nonzero_dir = 1;

        /* 死区处理 */
        if (raw_error >= -LINE_DEAD_ZONE && raw_error <= LINE_DEAD_ZONE)
            raw_error = 0;

        /* EMA 低通滤波 */
        filtered_error = (filtered_error + raw_error) >> LINE_FILTER_SHIFT;

        /* ---- 巡线 PD（内环）---- */
        int16_t line_pd = g_line_kp * filtered_error
                        + g_line_kd * (filtered_error - pid_last_error);

        /* ---- MPU 辅助（外环）---- */
        if (g_mpu_mode)
        {
            if (raw_error == 0)
            {
                /* ======================================================== */
                /*  居中直行：锁定航向 + 航向角 PID 稳航                        */
                /*  参考：电赛最常见的"锁定 yaw，PID 维持直行"方案              */
                /*  进入直道时锁一次 yaw，持续到转弯才释放                      */
                /* ======================================================== */
                if (!yaw_locked && center_cnt < YAW_LOCK_MS)
                    center_cnt++;
                if (!yaw_locked && center_cnt >= YAW_LOCK_MS)
                {
                    /* 首次达到阈值：锁定当前航向 + 重置 PID 状态 */
                    yaw_target = mpu_yaw;
                    yaw_locked = 1;
                    yaw_i = 0;
                    yaw_last_err = 0;
                }

                if (yaw_locked)
                {
                    float yaw_err = yaw_error_wrap(yaw_target, mpu_yaw);
                    int16_t yaw_out = yaw_pid_compute(yaw_err, mpu_gyro_z);
                    /* 灰度始终为主控，MPU 只占 30%，避免六轴 yaw 漂移抢控制权。 */
                    pid_output = (line_pd * 7 + yaw_out * 3) / 10;
                }
                else
                {
                    pid_output = line_pd;
                }
            }
            else
            {
                /* ======================================================== */
                /*  转弯中：释放航向锁 + 陀螺角速度连续阻尼                      */
                /*  参考：用 gyro_z 做连续阻尼，替代原来的分档硬切换            */
                /* ======================================================== */
                center_cnt = 0;
                yaw_locked = 0;
                yaw_i = 0.0f;
                yaw_last_err = 0.0f;

                /* 连续陀螺阻尼：实测逆时针 gyro_z 为正、顺时针为负。
                 * pid_output 正值对应顺时针，因此相加会削弱正在发生的转向，抑制甩尾。
                 * 用 |gyro_z|/15 做归一化因子：15°/s 以上全额阻尼，以下逐步减弱 */
                float abs_gyro = (mpu_gyro_z > 0) ? mpu_gyro_z : -mpu_gyro_z;
                float damping_scale = abs_gyro / 15.0f;
                if (damping_scale > 1.0f) damping_scale = 1.0f;
                int16_t gyro_damp = (int16_t)(mpu_gyro_z * YAW_GYRO_K * damping_scale);

                /* 灰度 PD 为主，陀螺项仅用于动态阻尼。 */
                pid_output = line_pd + gyro_damp;
            }
        }
        else
        {
            /* 非 MPU 模式：纯巡线 PD */
            pid_output = line_pd;
        }
    }

    pid_last_error = filtered_error;

    /* ========== 速度计算 + 限幅 + 输出 ========== */
    int16_t left_speed  = g_line_base_speed - pid_output;
    int16_t right_speed = g_line_base_speed + pid_output;

    if (left_speed > 500)  left_speed = 500;
    if (left_speed < MIN_FORWARD_SPEED) left_speed = MIN_FORWARD_SPEED;
    if (right_speed > 500)  right_speed = 500;
    if (right_speed < MIN_FORWARD_SPEED) right_speed = MIN_FORWARD_SPEED;

    Car_SetMotors(left_speed, right_speed);
}

/* ==================== 参数 getter/setter ==================== */
/* 供菜单系统运行时修改PD参数，无需重新编译 */

int16_t Line_GetKP(void)              { return g_line_kp; }
void    Line_SetKP(int16_t v)         { g_line_kp = v; }
int16_t Line_GetKD(void)              { return g_line_kd; }
void    Line_SetKD(int16_t v)         { g_line_kd = v; }
int16_t Line_GetLostGain(void)        { return g_line_lost_gain; }
void    Line_SetLostGain(int16_t v)   { g_line_lost_gain = v; }
int16_t Line_GetBaseSpeed(void)       { return g_line_base_speed; }
void    Line_SetBaseSpeed(int16_t v)  { g_line_base_speed = v; }
int16_t Line_GetPivotTime(void)       { return g_line_pivot_time; }
void    Line_SetPivotTime(int16_t v)  { g_line_pivot_time = v; }
int16_t Line_GetTurnSpeed(void)        { return g_line_turn_speed; }
void    Line_SetTurnSpeed(int16_t v)
{
    if (v < 80) v = 80;
    if (v > 250) v = 250;
    g_line_turn_speed = v;
}
uint8_t Line_GetMpuMode(void)         { return g_mpu_mode; }
void    Line_SetMpuMode(uint8_t v)
{
    uint8_t new_mode = v ? 1U : 0U;
    if (new_mode != g_mpu_mode)
    {
        g_mpu_mode = new_mode;
        yaw_locked = 0;
        center_cnt = 0;
        yaw_i = 0.0f;
        yaw_last_err = 0.0f;
        if (g_mpu_mode)
        {
            /* 每次开启辅助都从当前车体方向建立新的相对 yaw，避免沿用菜单等待期间的漂移。 */
            MPU6050_QuaternionReset();
            MPU6050_UpdateTimeReset();
        }
    }
}
