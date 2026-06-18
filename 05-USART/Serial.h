#include "ti_msp_dl_config.h"
#ifndef __SERIAL__H
#define __SERIAL__H

void UART_send_char(UART_Regs *uart, const uint8_t chr);

void UART_send_string(UART_Regs *uart, const char *str);


#endif
