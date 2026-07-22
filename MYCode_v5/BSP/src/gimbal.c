/* 云台舵机 — 接收 K230 ASCII 角度, 直接驱动 (参照原版 gimbal.c 机械参数) */
#include "gimbal.h"
#include "uart.h"
#include "ti_msp_dl_config.h"

/* PWM: TIMA0 50Hz, period=25000, 比较值=25000-脉宽 */
#define PWM_MIN   625U
#define PWM_MAX  3125U
#define PWM_PERIOD 25000U

/* 机械参数 — 不改你原来的值 */
#define CTR_H   900U   /* 水平中心 90.0° */
#define CTR_V   900U   /* 垂直中心 90.0° */
#define LIM_H   750U   /* 水平 ±75° */
#define LIM_V   600U   /* 垂直 ±60° */
#define LOST_MS 500U

static uint16_t g_h = CTR_H, g_v = CTR_V;
static uint8_t  g_lost = 1U;
static uint32_t g_ms = 0U, g_last = 0U;
static uint8_t  g_fcnt = 0U, g_fps = 0U;
static uint32_t g_fms = 0U;
static uint16_t g_total = 0U;

static uint16_t clamp16(uint16_t v, uint16_t lo, uint16_t hi) {
    if (v < lo) return lo; if (v > hi) return hi; return v;
}

static uint16_t a2p(uint16_t x10) {
    x10 = clamp16(x10, 0U, 1800U);
    uint32_t pulse = PWM_MIN + (uint32_t)(PWM_MAX - PWM_MIN) * x10 / 1800U;
    return (uint16_t)(PWM_PERIOD - pulse);
}

static void s_set(uint16_t h, uint16_t v) {
    DL_TimerA_setCaptureCompareValue(PWM_Servo_INST, h, GPIO_PWM_Servo_C1_IDX);
    DL_TimerA_setCaptureCompareValue(PWM_Servo_INST, v, GPIO_PWM_Servo_C0_IDX);
}

void Gimbal_Init(void) {
    DL_TimerA_startCounter(PWM_Servo_INST);
    s_set(a2p(CTR_H), a2p(CTR_V));
    g_h = CTR_H; g_v = CTR_V;
    g_lost = 1U; g_ms = 0U; g_last = 0U;
    UART_Init();
}

void Gimbal_Process(void) {
    g_ms++;
    UART_Poll();

    if (g_rx.fresh) {
        g_rx.fresh = false;
        /* 接收 K230 发来的绝对角度 (0.1°), clamp 到机械限位 */
        g_h = clamp16((uint16_t)(int16_t)g_rx.x_raw, CTR_H - LIM_H, CTR_H + LIM_H);
        g_v = clamp16((uint16_t)(int16_t)g_rx.y_raw, CTR_V - LIM_V, CTR_V + LIM_V);
        g_lost = 0U; g_last = g_ms;
        g_fcnt++; g_total++;
    }

    if ((g_ms - g_last) > LOST_MS) g_lost = 1U;

    if (!g_lost) s_set(a2p(g_h), a2p(g_v));

    if ((g_ms - g_fms) >= 500U) {
        g_fps = g_fcnt * 2U; g_fcnt = 0U; g_fms = g_ms;
    }
}

void Gimbal_Center(void) {
    s_set(a2p(CTR_H), a2p(CTR_V));
    g_h = CTR_H; g_v = CTR_V; g_lost = 1U;
}

uint16_t Gimbal_GetAngleH(void)  { return g_h / 10U; }
uint16_t Gimbal_GetAngleV(void)  { return g_v / 10U; }
int16_t  Gimbal_GetRelH(void)    { return (int16_t)(CTR_H - g_h) / 10; }
int16_t  Gimbal_GetRelV(void)    { return (int16_t)(g_v - CTR_V) / 10; }
uint16_t Gimbal_GetTargetH(void) { return g_h; }
uint16_t Gimbal_GetTargetV(void) { return g_v; }
uint8_t  Gimbal_IsLost(void)     { return g_lost; }
uint8_t  Gimbal_GetFPS(void)     { return g_fps; }
uint16_t Gimbal_GetFrameCnt(void){ return g_total; }
uint8_t  Gimbal_GetPage(void)    { return 0U; }
void     Gimbal_NextPage(void)   {}
bool     Gimbal_ParamSave(void)  { return true; }

void Gimbal_MoveH(float delta) {
    int16_t v = (int16_t)g_h + (int16_t)(delta * 10.0f);
    g_h = clamp16((uint16_t)(v > 0 ? v : 0), CTR_H - LIM_H, CTR_H + LIM_H);
    g_lost = 0U; s_set(a2p(g_h), a2p(g_v));
}
void Gimbal_MoveV(float delta) {
    int16_t v = (int16_t)g_v + (int16_t)(delta * 10.0f);
    g_v = clamp16((uint16_t)(v > 0 ? v : 0), CTR_V - LIM_V, CTR_V + LIM_V);
    g_lost = 0U; s_set(a2p(g_h), a2p(g_v));
}
