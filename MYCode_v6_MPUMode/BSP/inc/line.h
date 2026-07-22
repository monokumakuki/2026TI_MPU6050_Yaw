#ifndef _LINE_H
#define _LINE_H

#include "board.h"

/******************************************************************
 * 巡线模块 - 八路灰度传感器加权位置巡线
 * 传感器排列（从左到右）：
 *   LEFT1 LEFT2 LEFT3 LEFT4 RIGHT1 RIGHT2 RIGHT3 RIGHT4
 *   传感器检测到黑线时输出高电平
******************************************************************/

/* 传感器数量 */
#define LINE_SENSOR_COUNT 8

/* 小车运动状态枚举 */
typedef enum {
    DIR_FORWARD = 0,    /* 直行 */
    DIR_LEFT_SLIGHT,    /* 微左转 */
    DIR_LEFT_MEDIUM,    /* 中左转 */
    DIR_LEFT_SHARP,     /* 急左转 */
    DIR_RIGHT_SLIGHT,   /* 微右转 */
    DIR_RIGHT_MEDIUM,   /* 中右转 */
    DIR_RIGHT_SHARP,    /* 急右转 */
    DIR_LOST            /* 丢线 */
} CarState_t;

/******************************************************************
 * 函 数 名 称：Car_Forward
 * 函 数 说 明：小车前进
 * 函 数 形 参：speedL - 左电机速度(0~999)  speedR - 右电机速度(0~999)
 * 函 数 返 回：无
******************************************************************/
void Car_Forward(uint32_t speedL, uint32_t speedR);

/******************************************************************
 * 函 数 名 称：Car_TurnLeft
 * 函 数 说 明：小车左转（左轮反转，右轮反转）
 * 函 数 形 参：speedL - 左电机速度(0~999)  speedR - 右电机速度(0~999)
 * 函 数 返 回：无
******************************************************************/
void Car_TurnLeft(uint32_t speedL, uint32_t speedR);

/******************************************************************
 * 函 数 名 称：Car_TurnRight
 * 函 数 说 明：小车右转（左轮正转，右轮正转）
 * 函 数 形 参：speedL - 左电机速度(0~999)  speedR - 右电机速度(0~999)
 * 函 数 返 回：无
******************************************************************/
void Car_TurnRight(uint32_t speedL, uint32_t speedR);

/******************************************************************
 * 函 数 名 称：Car_Stop
 * 函 数 说 明：小车停止（刹车模式，两输入同高）
 * 函 数 形 参：无
 * 函 数 返 回：无
******************************************************************/
void Car_Stop(void);

void Car_SetMotors(int16_t speedL, int16_t speedR);

/******************************************************************
 * 函 数 名 称：Line_Tracking_Init
 * 函 数 说 明：巡线模块初始化，停车并重置状态
 * 函 数 形 参：无
 * 函 数 返 回：无
******************************************************************/
void Line_Tracking_Init(void);

/******************************************************************
 * 函 数 名 称：Line_ReadSensors
 * 函 数 说 明：读取八路传感器状态
 * 函 数 形 参：无
 * 函 数 返 回：uint8_t 8位掩码，bit0=LEFT1 ~ bit7=RIGHT4
******************************************************************/
uint8_t Line_ReadSensors(void);

/******************************************************************
 * 函 数 名 称：Line_GetPosition
 * 函 数 说 明：根据传感器状态计算加权位置值
 * 函 数 形 参：sensors - 传感器掩码
 * 函 数 返 回：int16_t 位置值，负值偏左，正值偏右，0居中
******************************************************************/
int16_t Line_GetPosition(uint8_t sensors);

/******************************************************************
 * 函 数 名 称：Line_GetState
 * 函 数 说 明：根据传感器状态判断小车运动方向
 * 函 数 形 参：无
 * 函 数 返 回：CarState_t 运动状态
******************************************************************/
CarState_t Line_GetState(void);

/******************************************************************
 * 函 数 名 称：Line_Execute
 * 函 数 说 明：根据运动状态执行对应的电机控制
 * 函 数 形 参：state - 运动状态
 * 函 数 返 回：无
******************************************************************/
void Line_Execute(CarState_t state);

/******************************************************************
 * 函 数 名 称：Line_Run
 * 函 数 说 明：巡线主循环单次执行，含状态滞后防抖
 * 函 数 形 参：无
 * 函 数 返 回：无
******************************************************************/
void Line_Run(void);

int16_t Line_GetKP(void);
void    Line_SetKP(int16_t v);
int16_t Line_GetKD(void);
void    Line_SetKD(int16_t v);
int16_t Line_GetLostGain(void);
void    Line_SetLostGain(int16_t v);
int16_t Line_GetBaseSpeed(void);
void    Line_SetBaseSpeed(int16_t v);
int16_t Line_GetPivotTime(void);
void    Line_SetPivotTime(int16_t v);
int16_t Line_GetTurnSpeed(void);
void    Line_SetTurnSpeed(int16_t v);
int16_t Line_GetYawKp10(void);
void    Line_SetYawKp10(int16_t v);
int16_t Line_GetYawRate100(void);
void    Line_SetYawRate100(int16_t v);
int16_t Line_GetMpuMix(void);
void    Line_SetMpuMix(int16_t v);
int16_t Line_GetYawLimit(void);
void    Line_SetYawLimit(int16_t v);
int16_t Line_GetYawLockCount(void);
void    Line_SetYawLockCount(int16_t v);
int16_t Line_GetCurveRate100(void);
void    Line_SetCurveRate100(int16_t v);
uint8_t Line_GetMpuMode(void);
void    Line_SetMpuMode(uint8_t v);

#endif
