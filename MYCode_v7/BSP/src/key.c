#include "key.h"
#include "ti_msp_dl_config.h"

#define KEY_DEBOUNCE_MS       20
#define KEY_LONG_PRESS_MS    500

/******************************************************************
 * 函 数 名 称：KEY_Init
 * 函 数 说 明：按键初始化（引脚由SysConfig配置，此处预留）
 * 函 数 形 参：无
 * 函 数 返 回：无
******************************************************************/
void KEY_Init(void)
{
    /* GPIO引脚已在SysConfig中配置为上拉输入，无需额外初始化 */
}

/******************************************************************
 * 函 数 名 称：KEY_IsPressed
 * 函 数 说 明：判断指定按键是否按下（无消抖，即时读取）
 * 函 数 形 参：key - 按键编号 KEY_1 或 KEY_2
 * 函 数 返 回：1-按下 0-未按下
******************************************************************/
uint8_t KEY_IsPressed(KeyNum_t key)
{
    switch (key)
    {
        case KEY_1:
            return (DL_GPIO_readPins(KEY_KEY1_PORT, KEY_KEY1_PIN) == 0) ? 1 : 0;
        case KEY_2:
            return (DL_GPIO_readPins(KEY_KEY2_PORT, KEY_KEY2_PIN) == 0) ? 1 : 0;
        case KEY_3:
            return (DL_GPIO_readPins(KEY_KEY3_PORT, KEY_KEY3_PIN) == 0) ? 1 : 0;
        case KEY_4:
            return (DL_GPIO_readPins(KEY_KEY4_PORT, KEY_KEY4_PIN) == 0) ? 1 : 0;
        default:
            return 0;
    }
}

/******************************************************************
 * 函 数 名 称：KEY_Scan
 * 函 数 说 明：扫描按键状态，带软件消抖
 * 函 数 形 参：无
 * 函 数 返 回：KeyNum_t 按键编号，KEY_NONE表示无按键按下
 * 备       注：检测到按键按下后会等待按键释放再返回
******************************************************************/
KeyNum_t KEY_Scan(void)
{
    /* 检测KEY1 */
    if (KEY_IsPressed(KEY_1))
    {
        delay_ms(KEY_DEBOUNCE_MS);
        if (KEY_IsPressed(KEY_1))
        {
            KEY_WaitRelease(KEY_1);
            return KEY_1;
        }
    }

    /* 检测KEY2 */
    if (KEY_IsPressed(KEY_2))
    {
        delay_ms(KEY_DEBOUNCE_MS);
        if (KEY_IsPressed(KEY_2))
        {
            KEY_WaitRelease(KEY_2);
            return KEY_2;
        }
    }

    return KEY_NONE;
}

/******************************************************************
 * 函 数 名 称：KEY_WaitRelease
 * 函 数 说 明：等待指定按键释放
 * 函 数 形 参：key - 按键编号 KEY_1 或 KEY_2
 * 函 数 返 回：无
******************************************************************/
void KEY_WaitRelease(KeyNum_t key)
{
    while (KEY_IsPressed(key))
        ;
}

uint8_t KEY2_ScanShortLong(void)
{
    if (!KEY_IsPressed(KEY_2))
        return KEY_EVT_NONE;

    delay_ms(KEY_DEBOUNCE_MS);
    if (!KEY_IsPressed(KEY_2))
        return KEY_EVT_NONE;

    uint16_t hold = 0;
    while (KEY_IsPressed(KEY_2) && hold < KEY_LONG_PRESS_MS)
    {
        delay_ms(1);
        hold++;
    }

    if (hold >= KEY_LONG_PRESS_MS)
    {
        while (KEY_IsPressed(KEY_2))
            ;
        return KEY_EVT_LONG;
    }

    return KEY_EVT_SHORT;
}

uint8_t KEY1_ScanShortLong(void)
{
    if (!KEY_IsPressed(KEY_1))
        return KEY_EVT_NONE;

    delay_ms(KEY_DEBOUNCE_MS);
    if (!KEY_IsPressed(KEY_1))
        return KEY_EVT_NONE;

    uint16_t hold = 0;
    while (KEY_IsPressed(KEY_1) && hold < KEY_LONG_PRESS_MS)
    {
        delay_ms(1);
        hold++;
    }

    if (hold >= KEY_LONG_PRESS_MS)
    {
        while (KEY_IsPressed(KEY_1))
            ;
        return KEY_EVT_LONG;
    }

    return KEY_EVT_SHORT;
}

uint8_t KEY3_ScanShortLong(void)
{
    if (!KEY_IsPressed(KEY_3))
        return KEY_EVT_NONE;

    delay_ms(KEY_DEBOUNCE_MS);
    if (!KEY_IsPressed(KEY_3))
        return KEY_EVT_NONE;

    uint16_t hold = 0;
    while (KEY_IsPressed(KEY_3) && hold < KEY_LONG_PRESS_MS)
    {
        delay_ms(1);
        hold++;
    }

    if (hold >= KEY_LONG_PRESS_MS)
    {
        while (KEY_IsPressed(KEY_3))
            ;
        return KEY_EVT_LONG;
    }

    return KEY_EVT_SHORT;
}

uint8_t KEY4_ScanShortLong(void)
{
    if (!KEY_IsPressed(KEY_4))
        return KEY_EVT_NONE;

    delay_ms(KEY_DEBOUNCE_MS);
    if (!KEY_IsPressed(KEY_4))
        return KEY_EVT_NONE;

    uint16_t hold = 0;
    while (KEY_IsPressed(KEY_4) && hold < KEY_LONG_PRESS_MS)
    {
        delay_ms(1);
        hold++;
    }

    if (hold >= KEY_LONG_PRESS_MS)
    {
        while (KEY_IsPressed(KEY_4))
            ;
        return KEY_EVT_LONG;
    }

    return KEY_EVT_SHORT;
}
