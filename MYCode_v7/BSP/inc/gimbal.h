#ifndef _GIMBAL_H
#define _GIMBAL_H

#include <stdint.h>
#include <stdbool.h>

/******************************************************************
 * K230 视觉追踪 → 舵机控制 (基于 servo_k230.h 参考实现)
 *
 * 通信: K230 UART → MSPM0G UART3(PB3=RX, PB2=TX)
 * 舵机: TIMA0 50Hz PWM, PB8=Tilt俯仰, PB9=Pan水平
 *
 * 帧协议 (10字节):
 *   [0xA5][0x5A][CMD][PAN_L][PAN_H][TILT_L][TILT_H][0][0][XOR]
 *   PAN/TILT 单位 0.1°, 小端序 int16
 *   CMD: 0x01=目标, 0x04=丢失
 *
 * API 保持与 empty.c 兼容
 ******************************************************************/

void Gimbal_Init(void);
void Gimbal_Process(void);       /* 主循环调用, 1ms一次 */
void Gimbal_Center(void);

/* 当前角度 (度) */
uint16_t Gimbal_GetAngleH(void);
uint16_t Gimbal_GetAngleV(void);
int16_t  Gimbal_GetRelH(void);
int16_t  Gimbal_GetRelV(void);

/* 目标角度 (0.1° 单位) */
uint16_t Gimbal_GetTargetH(void);
uint16_t Gimbal_GetTargetV(void);

/* 状态 */
uint8_t  Gimbal_IsLost(void);    /* 1=丢失(>500ms无帧) */
uint8_t  Gimbal_GetFPS(void);    /* 帧率 */

/* 统计 */
uint16_t Gimbal_GetFrameCnt(void);

/* 手动控制 (舵机测试用) */
void Gimbal_MoveH(float delta);
void Gimbal_MoveV(float delta);

/* OLED 页面 (简化: 只有1页) */
uint8_t Gimbal_GetPage(void);
void    Gimbal_NextPage(void);

/* Flash 存取 */
bool Gimbal_ParamSave(void);

#endif
