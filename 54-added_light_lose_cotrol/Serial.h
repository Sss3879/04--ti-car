#include "ti_msp_dl_config.h"
#include <stdint.h>
#ifndef __SERIAL__H
#define __SERIAL__H

extern volatile uint8_t Serial_RxFlag;
extern char Serial_RxPacket[64];

/* 接收计数器：每收到一包有效 K230 数据就 +1，用于超时检测 */
extern volatile uint32_t Serial_RxCounter;

void UART_send_char(UART_Regs *uart, const uint8_t chr);

void UART_send_string(UART_Regs *uart, const char *str);

void Serial_Printf(const char *format, ...);

int16_t Serial_GetXError(void);

int16_t Serial_GetYError(void);

uint32_t Serial_GetRxCounter(void);

#endif
