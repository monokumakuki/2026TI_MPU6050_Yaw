#include "encoder.h"
#include "ti_msp_dl_config.h"

/* 左右编码器累计计数值 */
static volatile int32_t encoder_left_count = 0;
static volatile int32_t encoder_right_count = 0;

/******************************************************************
 * 函 数 名 称：Encoder_Init
 * 函 数 说 明：编码器初始化，配置GPIO外部中断并清零计数值
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：A相配置为双沿触发中断，在ISR中读取B相判断方向
 *             双沿检测实现2倍频，比原轮询方式(仅上升沿)精度翻倍
 ******************************************************************/
void Encoder_Init(void)
{
    encoder_left_count = 0;
    encoder_right_count = 0;

    /* 设置A相双沿触发极性
     * Right_A = PA15 (lower pins [0,15])
     * Left_A  = PA17 (upper pins [16,31])
     */
    DL_GPIO_setLowerPinsPolarity(Encoder_PORT, DL_GPIO_PIN_15_EDGE_RISE_FALL);
    DL_GPIO_setUpperPinsPolarity(Encoder_PORT, DL_GPIO_PIN_17_EDGE_RISE_FALL);

    /* 使能左编码器A相中断 */
    DL_GPIO_enableInterrupt(Encoder_PORT, Encoder_Left_Encode_A_PIN);

    /* 使能右编码器A相中断 */
    DL_GPIO_enableInterrupt(Encoder_PORT, Encoder_Right_Encode_A_PIN);

    /* 清除可能存在的挂起中断标志 */
    DL_GPIO_clearInterruptStatus(Encoder_PORT,
        Encoder_Left_Encode_A_PIN | Encoder_Right_Encode_A_PIN);

    /* 使能GPIOA组中断 (GROUP1) */
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

/******************************************************************
 * 函 数 名 称：Encoder_GPIO_IRQHandler
 * 函 数 说 明：编码器GPIO中断处理，由GROUP1_IRQHandler调用
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：A相双沿触发中断，读取A/B相电平判断方向
 *             正交解码逻辑：
 *               A相跳变后，A==B 表示正转，A!=B 表示反转
 ******************************************************************/
void Encoder_GPIO_IRQHandler(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(Encoder_PORT,
        Encoder_Left_Encode_A_PIN | Encoder_Right_Encode_A_PIN);

    /* 左编码器A相中断 */
    if (pending & Encoder_Left_Encode_A_PIN)
    {
        uint8_t a = (DL_GPIO_readPins(Encoder_PORT, Encoder_Left_Encode_A_PIN) > 0) ? 1 : 0;
        uint8_t b = (DL_GPIO_readPins(Encoder_PORT, Encoder_Left_Encode_B_PIN) > 0) ? 1 : 0;

        if (a == b)
            encoder_left_count++;    /* A==B: 正转 */
        else
            encoder_left_count--;    /* A!=B: 反转 */

        DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_Left_Encode_A_PIN);
    }

    /* 右编码器A相中断 */
    if (pending & Encoder_Right_Encode_A_PIN)
    {
        uint8_t a = (DL_GPIO_readPins(Encoder_PORT, Encoder_Right_Encode_A_PIN) > 0) ? 1 : 0;
        uint8_t b = (DL_GPIO_readPins(Encoder_PORT, Encoder_Right_Encode_B_PIN) > 0) ? 1 : 0;

        if (a == b)
            encoder_right_count++;   /* A==B: 正转 */
        else
            encoder_right_count--;   /* A!=B: 反转 */

        DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_Right_Encode_A_PIN);
    }
}

/******************************************************************
 * 函 数 名 称：GROUP1_IRQHandler
 * 函 数 说 明：GPIOA组中断服务函数（编码器外部中断）
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：MSPM0G3507中GPIOA所有引脚中断共用此IRQHandler
 ******************************************************************/
void GROUP1_IRQHandler(void)
{
    Encoder_GPIO_IRQHandler();
}

/******************************************************************
 * 函 数 名 称：Encoder_GetLeft
 * 函 数 说 明：获取左编码器累计计数值
 * 函 数 形 参：无
 * 函 数 返 回：int32_t 左编码器计数值
******************************************************************/
int32_t Encoder_GetLeft(void)
{
    return encoder_left_count;
}

/******************************************************************
 * 函 数 名 称：Encoder_GetRight
 * 函 数 说 明：获取右编码器累计计数值
 * 函 数 形 参：无
 * 函 数 返 回：int32_t 右编码器计数值
******************************************************************/
int32_t Encoder_GetRight(void)
{
    return encoder_right_count;
}

/******************************************************************
 * 函 数 名 称：Encoder_Reset
 * 函 数 说 明：清零左右编码器计数值
 * 函 数 形 参：无
 * 函 数 返 回：无
******************************************************************/
void Encoder_Reset(void)
{
    encoder_left_count = 0;
    encoder_right_count = 0;
}

/******************************************************************
 * 函 数 名 称：Encoder_GetBoth
 * 函 数 说 明：同时获取左右编码器计数值
 * 函 数 形 参：*left - 左编码器计数值指针
 *             *right - 右编码器计数值指针
 * 函 数 返 回：无
******************************************************************/
void Encoder_GetBoth(int32_t *left, int32_t *right)
{
    *left = encoder_left_count;
    *right = encoder_right_count;
}

/******************************************************************
 * 函 数 名 称：Encoder_IsLapComplete
 * 函 数 说 明：判断是否已完成指定圈数
 * 函 数 形 参：target_laps - 目标圈数
 * 函 数 返 回：1-已完成 0-未完成
 * 备       注：取左右编码器绝对值的平均值与目标脉冲数比较
 *             双沿检测下每圈脉冲数为原2倍，需将宏值除以2
******************************************************************/
uint8_t Encoder_IsLapComplete(uint8_t target_laps)
{
    /* 取绝对值，无论正转反转都计入圈数 */
    int32_t left_abs = (encoder_left_count < 0) ? -encoder_left_count : encoder_left_count;
    int32_t right_abs = (encoder_right_count < 0) ? -encoder_right_count : encoder_right_count;

    /* 取左右平均值作为实际行程 */
    int32_t avg = (left_abs + right_abs) / 2;
    /* 双沿检测使每圈计数翻倍，ENCODER_COUNTS_PER_LAP需乘2 */
    int32_t target = (int32_t)target_laps * ENCODER_COUNTS_PER_LAP * 2;

    return (avg >= target) ? 1 : 0;
}
