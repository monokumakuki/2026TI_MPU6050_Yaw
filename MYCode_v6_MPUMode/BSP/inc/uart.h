#ifndef _UART_H
#define _UART_H
#include <stdint.h>
#include <stdbool.h>

#define BUF_SZ 24

/* K230 → MSPM0G ASCII 协议: $PAN,TILT\r\n 或 $LOST\r\n
 * PAN/TILT = 0.1° 单位, 例 $900,850\r\n = 水平90.0° 垂直85.0° */
typedef struct {
    int16_t  x_raw, y_raw;
    bool     fresh;
    uint16_t cnt;
} Frame_t;

extern Frame_t g_rx;

void UART_Init(void);
void UART_Poll(void);
#endif
