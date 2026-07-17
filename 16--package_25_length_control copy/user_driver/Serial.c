#include "Serial.h"
#include <stdio.h>
#include <stdarg.h>
volatile uint8_t Serial_RxFlag = 0;
char Serial_RxPacket[64];

static uint8_t Serial_RxIndex = 0;
void UART_send_char(UART_Regs *uart, const uint8_t chr)
{
    DL_UART_transmitDataBlocking(uart, chr);
}

void UART_send_string(UART_Regs *uart, const char *str)
{
    while (*str) {
        UART_send_char(uart, (uint8_t) *str);
        str++;
    }
}
/**
 * @brief 串口格式化输出
 */
void Serial_Printf(const char *format, ...)
{
    char buffer[128];

    va_list args;
    va_start(args, format);

    vsnprintf(buffer, sizeof(buffer), format, args);

    va_end(args);

    UART_send_string(Serial_INST, buffer);
}

/* ========================== printf 重定向 (UART0) ========================== */
int fputc(int ch, FILE *stream)
{
    while (DL_UART_isBusy(Serial_INST));
    DL_UART_Main_transmitData(Serial_INST, ch);
    return ch;
}

int fputs(const char *restrict s, FILE *restrict stream)
{
    uint16_t len = 0;
    while (*s) {
        while (DL_UART_isBusy(Serial_INST));
        DL_UART_Main_transmitData(Serial_INST, *s++);
        len++;
    }
    return len;
}

int puts(const char *_ptr)
{
    return 0;
}

// void Serial_INST_IRQHandler()
// {
//     switch (DL_UART_getPendingInterrupt(Serial_INST))
//         {
//         case DL_UART_IIDX_RX:
//             {   
//                 uint8_t rec = DL_UART_receiveData(Serial_INST);
//                 UART_send_char(Serial_INST, rec);
//                 break;
//             }
        
//         default:
//             break;
//         }
// }

void Serial_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(Serial_INST))
    {
        case DL_UART_IIDX_RX:
        {
            char ch = DL_UART_receiveData(Serial_INST);

            // 如果上一包还没处理，就先不覆盖
            if (Serial_RxFlag == 0)
            {
                if (ch == '\n')
                {
                    Serial_RxPacket[Serial_RxIndex] = '\0';
                    Serial_RxIndex = 0;
                    Serial_RxFlag = 1;
                }
                else if (ch != '\r')
                {
                    if (Serial_RxIndex < sizeof(Serial_RxPacket) - 1)
                    {
                        Serial_RxPacket[Serial_RxIndex++] = ch;
                    }
                    else
                    {
                        // 超出长度，丢弃这一包
                        Serial_RxIndex = 0;
                    }
                }
            }

            break;
        }

        default:
            break;
    }
}
