#ifndef _ENCODER_H
#define _ENCODER_H

#include "board.h"

#define ENCODER_COUNTS_PER_LAP 3200   

void Encoder_Init(void);
int32_t Encoder_GetLeft(void);
int32_t Encoder_GetRight(void);
void Encoder_Reset(void);
void Encoder_GetBoth(int32_t *left, int32_t *right);
uint8_t Encoder_IsLapComplete(uint8_t target_laps);
void Encoder_GPIO_IRQHandler(void);

#endif
