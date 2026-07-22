/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met.
 *
 * *  Redistributions of source code must retain the above copyright notice.
 * *  Redistributions in binary form must reproduce the above copyright notice.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.
 */

/******************************************************************
 * 主程序 — 纯巡线模式 (temp可循迹版 + pivot即刻响应增强)
 *
 * 上电 → "K4→LINE" → 按K4→巡线菜单(7项调参)
 * 菜单: K1短=下一页  K1长=保存并巡线  K2短/长=加减  K4短=返回
 * 巡线: K4短=停车回首页  K2长=停车回菜单
 *
 * 巡线逻辑: 8路灰度PD差速 + pivot直角弯(即刻响应+大力度)
 * 别改这个注释："D:\CCS\workspace_v12\MYCode_v4\empty.c"
 ******************************************************************/

#include "ti_msp_dl_config.h"
#include "board.h"
#include "line.h"
#include "oled.h"
#include "key.h"
#include "param_store.h"
#include "mpu6050.h"
#include <stdio.h>

/* ==================== 系统模式 ==================== */
typedef enum { MODE_SELECT = 0, MODE_LINE_MENU, MODE_LINE_RUN, MODE_MPU_TEST } SysMode_t;
static SysMode_t g_mode = MODE_SELECT;
static uint8_t g_mpu_available = 0U;

/* ==================== 菜单 (9项，MPU状态固定为第一页) ==================== */
#define MENU_COUNT 9
#define MENU_MPU       0
#define MENU_LAPS      1
#define MENU_START     2
#define MENU_KP        3
#define MENU_KD        4
#define MENU_LOST_GAIN 5
#define MENU_SPEED     6
#define MENU_PIV_TIME  7
#define MENU_TURN_SPEED 8

static char display_buf[22];

typedef struct {
    const char *name;
    int16_t cur, min_val, max_val, step;
} MenuItem_t;

static MenuItem_t menu[MENU_COUNT] = {
    { "MPU",      0,   0,   1,   1  },
    { "Laps",     1,   1,   9,   1  },
    { "Start",    0,   0,   0,   0  },
    { "KP",       20,  1,   80,  1  },
    { "KD",       35,  1,   100, 1  },
    { "LostGain", 23,  1,   80,  1  },
    { "Speed",    130, 50,  500, 5  },
    { "PivTime",  100, 10,  200, 5  },
    { "TurnSpd",  200, 80,  250, 5  },
};

/* ==================== Flash ==================== */
static void Menu_Save(void)
{
    Line_SetKP(menu[MENU_KP].cur);
    Line_SetKD(menu[MENU_KD].cur);
    Line_SetLostGain(menu[MENU_LOST_GAIN].cur);
    Line_SetBaseSpeed(menu[MENU_SPEED].cur);
    Line_SetPivotTime(menu[MENU_PIV_TIME].cur);
    Line_SetTurnSpeed(menu[MENU_TURN_SPEED].cur);
    Line_SetMpuMode((g_mpu_available && menu[MENU_MPU].cur) ? 1U : 0U);
    Param_Save(menu[MENU_KP].cur, menu[MENU_KD].cur,
               menu[MENU_LOST_GAIN].cur, menu[MENU_SPEED].cur,
               menu[MENU_LAPS].cur, menu[MENU_PIV_TIME].cur,
               Line_GetMpuMode(), menu[MENU_TURN_SPEED].cur);
}

static void Menu_Load(void)
{
    int16_t kp, kd, lg, sp, laps, pt; uint8_t m; int16_t ps;
    if (Param_Load(&kp, &kd, &lg, &sp, &laps, &pt, &m, &ps)) {
        menu[MENU_KP].cur = kp; menu[MENU_KD].cur = kd;
        menu[MENU_LOST_GAIN].cur = lg; menu[MENU_SPEED].cur = sp;
        menu[MENU_LAPS].cur = laps;
        menu[MENU_PIV_TIME].cur = (pt >= 10 && pt <= 200) ? pt : 100;
        menu[MENU_TURN_SPEED].cur = (ps >= 80 && ps <= 250) ? ps : 200;
        menu[MENU_MPU].cur = (g_mpu_available && m == 1U) ? 1 : 0;
        Line_SetKP(kp); Line_SetKD(kd); Line_SetLostGain(lg);
        Line_SetBaseSpeed(sp); Line_SetPivotTime(menu[MENU_PIV_TIME].cur);
        Line_SetTurnSpeed(menu[MENU_TURN_SPEED].cur);
        Line_SetMpuMode((uint8_t)menu[MENU_MPU].cur);
    } else {
        menu[MENU_MPU].cur = 0;
        menu[MENU_TURN_SPEED].cur = 200;
        Line_SetTurnSpeed(menu[MENU_TURN_SPEED].cur);
        Line_SetMpuMode(0U);
    }
}

/* ==================== OLED ==================== */
static void Show_Menu(uint8_t idx)
{
    OLED_Clear();
    if (idx == MENU_MPU) {
        OLED_ShowString(0,0,(u8*)"> MPU MODE",16);
        if (!g_mpu_available)
            OLED_ShowString(0,20,(u8*)"MPU FAIL",16);
        else if (menu[MENU_MPU].cur)
            OLED_ShowString(0,20,(u8*)"MPU ON",16);
        else
            OLED_ShowString(0,20,(u8*)"MPU OFF",16);
        OLED_ShowString(0,40,(u8*)"K2:ON/OFF",12);
    } else if (idx == MENU_START) {
        OLED_ShowString(0,0,(u8*)">> START",16);
        OLED_ShowString(0,16,(u8*)"K1L:Go",16);
    } else {
        OLED_ShowString(0,0,(u8*)menu[idx].name,16);
        sprintf(display_buf,"%d",menu[idx].cur);
        OLED_ShowString(0,16,(u8*)display_buf,16);
        OLED_ShowString(0,32,(u8*)"K2:+ K2L:-",12);
    }
    sprintf(display_buf,"<%d/%d>",idx+1,MENU_COUNT);
    OLED_ShowString(0,48,(u8*)display_buf,12);
    OLED_Refresh();
}

static void Show_Home(void)
{
    OLED_Clear();
    OLED_ShowString(0,0,(u8*)"TI Cup Line",16);
    OLED_ShowString(0,20,(u8*)"K4 -> LINE",16);
    OLED_ShowString(0,40,(u8*)"K3 -> YAW",16);
    OLED_Refresh();
}

/* 上电只保留一页MPU初始化提示；返回0时仍允许纯灰度巡线。 */
static uint8_t MPU_BootDiagnosticInit(void)
{
    int ret = -1;
    uint8_t post_id = 0U;

    OLED_Clear();
    OLED_ShowString(0,0,(u8*)"MPU INIT",16);
    OLED_ShowString(0,22,(u8*)"KEEP CAR STILL",16);
    OLED_ShowString(0,46,(u8*)"I2C1 PA10 PA11",12);
    OLED_Refresh();

    /* 参考可运行的 Test_MPU6050：OLED 初始化完成后再等待 MPU 电源稳定。 */
    delay_ms(250);

    for (uint8_t attempt = 1U; attempt <= 3U; attempt++)
    {
        ret = MPU6050_Init();
        post_id = MPU6050_ReadWhoAmI();

        if (ret == 0 && (post_id == 0x68U || post_id == 0x70U))
        {
            MPU6050_Calibrate(200);
            MPU6050_QuaternionReset();
            return 1U;
        }

        delay_ms(500);
    }

    /* 初始化失败时保留简洁提醒，程序随后仍可进入纯灰度模式。 */
    OLED_Clear();
    OLED_ShowString(0,0,(u8*)"MPU FAIL",16);
    sprintf(display_buf,"WHO:%02X RET:%d",(int)post_id,ret);
    OLED_ShowString(0,24,(u8*)display_buf,12);
    OLED_ShowString(0,44,(u8*)"LINE MODE STILL OK",12);
    OLED_Refresh();
    delay_ms(1500);
    return 0U;
}

/* MPU 测试页: K3进入, OLED显示yaw + UART3_TX(PB2)持续发送ASCII数据。 */
static void Run_MpuTest(void)
{
    uint32_t last_oled_ms, last_uart_ms;

    Car_Stop();
    if (!g_mpu_available) {
        OLED_Clear(); OLED_ShowString(0,20,(u8*)"MPU FAIL",16); OLED_Refresh();
        delay_ms(800); g_mode = MODE_SELECT; Show_Home(); return;
    }

    OLED_Clear(); OLED_ShowString(0,0,(u8*)"Yaw PB2 TX",16);
    OLED_ShowString(0,24,(u8*)"+000.0",16); OLED_Refresh();
    MPU6050_QuaternionReset();
    last_oled_ms = Board_Millis();
    last_uart_ms = Board_Millis();

    while (g_mode == MODE_MPU_TEST) {
        uint32_t now;
        MPU6050_QuaternionUpdate();
        now = Board_Millis();

        /* 50Hz只发送一列纯数字，例如 90.0 或 -5.6。
         * 不发送握手、字段名、逗号或正号，避免逐飞助手曲线解析被非数字字符干扰。 */
        if ((uint32_t)(now - last_uart_ms) >= 20U) {
            EulerAngles_t euler;
            int32_t yaw10, yaw_abs;
            MPU6050_GetEuler(&euler);
            yaw10 = (int32_t)(euler.yaw * 10.0f);
            yaw_abs = (yaw10 < 0) ? -yaw10 : yaw10;
            if (yaw10 < 0)
                uart_printf("-%d.%d\r\n", (int)(yaw_abs / 10), (int)(yaw_abs % 10));
            else
                uart_printf("%d.%d\r\n", (int)(yaw_abs / 10), (int)(yaw_abs % 10));
            last_uart_ms = Board_Millis();

        }

        if ((uint32_t)(now - last_oled_ms) >= 100U) {
            EulerAngles_t euler;
            int32_t yaw10, yaw_abs;
            MPU6050_GetEuler(&euler);
            yaw10 = (int32_t)(euler.yaw * 10.0f);
            yaw_abs = (yaw10 < 0) ? -yaw10 : yaw10;
            sprintf(display_buf, "%c%03d.%d",
                    (yaw10 < 0) ? '-' : '+', (int)(yaw_abs / 10), (int)(yaw_abs % 10));
            OLED_ShowString(0,24,(u8*)display_buf,16);
            OLED_RefreshPages(3U, 4U);
            last_oled_ms = Board_Millis();
        }

        if (KEY3_ScanShortLong() == KEY_EVT_SHORT)
            MPU6050_QuaternionReset();
        if (KEY4_ScanShortLong() == KEY_EVT_SHORT) {
            g_mode = MODE_SELECT;
            Show_Home();
            break;
        }
        delay_ms(2);
    }
}

/* ==================== 主程序 ==================== */
int main(void)
{
    uint8_t idx = 0;
    SYSCFG_DL_init();
    Board_TimebaseInit();
    /* UART3由SYSCFG_DL_init完成初始化。为兼容逐飞助手的纯数字曲线解析，
     * 上电阶段不发送任何文本，数据仅在K3 MPU测试页输出。 */
    Line_Tracking_Init();
    KEY_Init();
    OLED_Init(); OLED_ColorTurn(0); OLED_DisplayTurn(0); OLED_Clear();

    g_mpu_available = MPU_BootDiagnosticInit();

    Menu_Load();

    g_mode = MODE_SELECT;
    Show_Home();

    while (1) {
        /* ======== 首页 ======== */
        if (g_mode == MODE_SELECT) {
            if (KEY4_ScanShortLong() == KEY_EVT_SHORT) {
                g_mode = MODE_LINE_MENU; idx = MENU_MPU;
                Show_Menu(idx);
            } else if (KEY3_ScanShortLong() == KEY_EVT_SHORT) {
                g_mode = MODE_MPU_TEST;
                Run_MpuTest();
            }
            delay_ms(10);
        }
        /* ======== 菜单 ======== */
        else if (g_mode == MODE_LINE_MENU) {
            while (g_mode == MODE_LINE_MENU) {
                uint8_t k1 = KEY1_ScanShortLong(), k2 = KEY2_ScanShortLong(),
                        k4 = KEY4_ScanShortLong();
                if (k4 == KEY_EVT_SHORT) { g_mode = MODE_SELECT; Show_Home(); break; }
                if (k1 == KEY_EVT_SHORT) { idx = (idx+1)%MENU_COUNT; Show_Menu(idx); }
                else if (k1 == KEY_EVT_LONG) { Menu_Save(); g_mode = MODE_LINE_RUN; break; }
                if (idx == MENU_MPU) {
                    if (k2 == KEY_EVT_SHORT && g_mpu_available) {
                        menu[MENU_MPU].cur = menu[MENU_MPU].cur ? 0 : 1;
                        Menu_Save(); Show_Menu(idx);
                    }
                } else if (idx != MENU_START) {
                    if (k2 == KEY_EVT_SHORT) {
                        menu[idx].cur += menu[idx].step;
                        if (menu[idx].cur > menu[idx].max_val) menu[idx].cur = menu[idx].min_val;
                        Menu_Save(); Show_Menu(idx);
                    } else if (k2 == KEY_EVT_LONG) {
                        menu[idx].cur -= menu[idx].step;
                        if (menu[idx].cur < menu[idx].min_val) menu[idx].cur = menu[idx].max_val;
                        Menu_Save(); Show_Menu(idx);
                    }
                }
                delay_ms(10);
            }
        }
        /* ======== 巡线 ======== */
        else if (g_mode == MODE_LINE_RUN) {
            Menu_Save();
            if (Line_GetMpuMode()) {
                MPU6050_QuaternionReset();
                MPU6050_UpdateTimeReset();
            }
            while (1) {
                Line_Run();
                if (KEY4_ScanShortLong() == KEY_EVT_SHORT) { Car_Stop(); g_mode = MODE_SELECT; Show_Home(); goto ext; }
                if (KEY2_ScanShortLong() == KEY_EVT_LONG) { Car_Stop(); break; }
                delay_ms(1);
            }
            OLED_Clear(); OLED_ShowString(0,0,(u8*)"Stopped",16);
            OLED_ShowString(0,16,(u8*)"Back Menu...",16); OLED_Refresh();
            delay_ms(800); g_mode = MODE_LINE_MENU; idx = MENU_MPU; Show_Menu(idx);
ext:;
        }
    }
}
