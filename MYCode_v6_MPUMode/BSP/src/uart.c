/* K230 → MSPM0G ASCII 协议 — 已验证能用的版本 */
#include "uart.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>

Frame_t g_rx = {0};

static char buf[BUF_SZ];
static uint8_t idx = 0;
static bool esc = false;

static int16_t atoi2(const char *s, uint8_t n) {
    bool neg = false; int16_t v = 0; uint8_t i = 0;
    if (i < n && s[i] == '-') { neg = true; i++; }
    for (; i < n && s[i] >= '0' && s[i] <= '9'; i++)
        v = v * 10 + (int16_t)(s[i] - '0');
    return neg ? -v : v;
}

static void proc(uint8_t byte) {
    char c = (char)byte;
    if (!esc) { if (c == '$') { esc = true; idx = 0; } return; }
    if (c == '\r' || c == '\n') {
        if (idx >= 4 && buf[0] == 'L' && buf[1] == 'O' && buf[2] == 'S' && buf[3] == 'T') {
            g_rx.x_raw = 0; g_rx.y_raw = 0; g_rx.fresh = true; g_rx.cnt++;
        } else if (idx > 0) {
            uint8_t comma = 0xFF;
            for (uint8_t i = 0; i < idx; i++)
                if (buf[i] == ',') { comma = i; break; }
            if (comma != 0xFF && comma > 0 && comma < idx - 1) {
                g_rx.x_raw = atoi2(buf, comma);
                g_rx.y_raw = atoi2(buf + comma + 1, idx - comma - 1);
                g_rx.fresh = true; g_rx.cnt++;
            }
        }
        esc = false; idx = 0;
        return;
    }
    if (idx < BUF_SZ - 1) buf[idx++] = c;
    else { esc = false; idx = 0; }
}

void UART3_IRQHandler(void) {
    proc(DL_UART_Main_receiveData(UART_Cam_INST));
}

void UART_Poll(void) {
    while (!DL_UART_Main_isRXFIFOEmpty(UART_Cam_INST))
        proc(DL_UART_Main_receiveData(UART_Cam_INST));
}

void UART_Init(void) {
    DL_UART_Main_enableFIFOs(UART_Cam_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_Cam_INST, DL_UART_MAIN_RX_FIFO_LEVEL_1_2_FULL);
    DL_UART_Main_enableInterrupt(UART_Cam_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_Cam_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_Cam_INST_INT_IRQN);
}
