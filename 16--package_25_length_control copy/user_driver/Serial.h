#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdio.h>
#ifndef __SERIAL__H
#define __SERIAL__H

extern volatile uint8_t Serial_RxFlag;
extern char Serial_RxPacket[64];

void UART_send_char(UART_Regs *uart, const uint8_t chr);

void UART_send_string(UART_Regs *uart, const char *str);

void Serial_Printf(const char *format, ...);

/* printf 重定向到 UART0 */
int fputc(int ch, FILE *stream);
int fputs(const char *restrict s, FILE *restrict stream);
int puts(const char *_ptr);

#endif
