#include "Serial.h"
#include <stdio.h>
#include <stdarg.h>
volatile uint8_t Serial_RxFlag = 0;
char Serial_RxPacket[64];

volatile uint32_t Serial_RxCounter = 0;  /* 每收到一包有效数据就 +1 */

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


int16_t Serial_XError = 0;                  // K230 发来的 x_error
int16_t Serial_YError = 0;                  // K230 发来的 y_error


int16_t Serial_GetXError(void)
{
	return Serial_XError;
}

int16_t Serial_GetYError(void)
{
	return Serial_YError;
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

                    // 解析K230误差包: "-8,-2" -> x, y
                    int x, y;
                    if (sscanf(Serial_RxPacket, "%d,%d", &x, &y) == 2)
                    {
                        Serial_XError = (int16_t)x;
                        Serial_YError = (int16_t)y;
                        Serial_RxCounter++;  /* 有效数据包计数 */
                    }

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

/**
 * @brief 获取接收计数器值（用于超时检测）
 */
uint32_t Serial_GetRxCounter(void)
{
    return Serial_RxCounter;
}
