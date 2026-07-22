

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "board.h"
#include "ti/driverlib/m0p/dl_core.h"

/* SysTick 每 1ms 递增；只做计时，不在中断中执行外设或浮点运算。 */
static volatile uint32_t s_board_millis = 0U;


/* ================ UART3调试输出 =================== */

int fputc(int ch, FILE *f)
{
    return ch;
}

int LOG_Debug_Out(const char* __file, const char* __func, int __line, const char* format, ...)
{
    return 0;
}

#include "ti/driverlib/dl_uart_main.h"

/* UART3 发送 (PB2=TX, 已由 SYSCFG_DL_init 初始化, 115200) */
void uart_send_byte(char c)
{
    /* UART_Cam由SysConfig生成为UART Main实例。
     * 使用阻塞发送可保证前一个字节进入FIFO后再继续，避免测试数据丢字节。 */
    DL_UART_Main_transmitDataBlocking(UART_Cam_INST, (uint8_t)c);
}

/* UART3 发送字符串 */
void uart_send_str(const char *s)
{
    while (*s) uart_send_byte(*s++);
}

/* 简易 printf → UART3 */
static void uart_putnum(int n)
{
    char buf[12];
    int i = 0, neg = 0;
    if (n < 0) { neg = 1; n = -n; }
    do { buf[i++] = '0' + (n % 10); n /= 10; } while (n);
    if (neg) buf[i++] = '-';
    while (i--) uart_send_byte(buf[i]);
}

#include <stdarg.h>
int uart_printf(const char *fmt, ...)
{
    va_list ap;
    const char *p;
    va_start(ap, fmt);
    for (p = fmt; *p; p++) {
        if (*p == '%' && *(p+1)) {
            p++;
            switch (*p) {
            case 'd': uart_putnum(va_arg(ap, int)); break;
            case 'f': {
                float v = (float)va_arg(ap, double);
                int i = (int)v;
                uart_putnum(i);
                uart_send_byte('.');
                int frac = (int)((v - i) * 100.0f);
                if (frac < 0) frac = -frac;
                uart_putnum(frac);
                break;
            }
            case 's': uart_send_str(va_arg(ap, char*)); break;
            case 'c': uart_send_byte((char)va_arg(ap, int)); break;
            default:  uart_send_byte(*p); break;
            }
        } else {
            uart_send_byte(*p);
        }
    }
    va_end(ap);
    return 0;
}

int lc_printf(char* format,...)
{
    return 0;
}


/* ================ 延时函数封装 =================== */

void delay_us(int __us) { delay_cycles( (CPUCLK_FREQ / 1000 / 1000)*__us); }
void delay_ms(int __ms) { delay_cycles( (CPUCLK_FREQ / 1000)*__ms); }

void delay_1us(int __us) { delay_cycles( (CPUCLK_FREQ / 1000 / 1000)*__us); }
void delay_1ms(int __ms) { delay_cycles( (CPUCLK_FREQ / 1000)*__ms); }

void Board_TimebaseInit(void)
{
    s_board_millis = 0U;
    (void)SysTick_Config(CPUCLK_FREQ / 1000U);
}

uint32_t Board_Millis(void)
{
    return s_board_millis;
}

void SysTick_Handler(void)
{
    s_board_millis++;
}
