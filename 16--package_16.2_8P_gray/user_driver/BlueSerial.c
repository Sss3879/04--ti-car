/*******************************************************************************
 * @file        BlueSerial.c
 * @brief       TI MSPM0G3507 蓝牙串口驱动 (UART3, 9600bps)
 *
 *              - 移植自 STM32 BlueSerial，保持相同 API
 *              - 使用 TI DriverLib (DL_UART) 接口
 *              - 数据帧：'[' 帧头, ']' 帧尾
 *
 *              蓝牙模块接线：
 *                PA26  —  TX  → 蓝牙模块 RX
 *                PA25  —  RX  ← 蓝牙模块 TX
 *******************************************************************************/
#include "BlueSerial.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* 全局变量 */
char BlueSerial_RxPacket[BLUE_SERIAL_RX_PACKET_MAX_LEN];
volatile uint8_t BlueSerial_RxFlag = 0;

/* ======== 初始化 ======== */
void BlueSerial_Init(void)
{
    /*
     * SYSCFG_DL_init() 已经配置好 UART3 引脚、时钟和波特率(9600)。
     * 这里只需使能 UART3 的 RX 中断。
     */
    NVIC_ClearPendingIRQ(Blue_Serial_INST_INT_IRQN);
    NVIC_EnableIRQ(Blue_Serial_INST_INT_IRQN);
}

/* ======== 字节发送 ======== */
void BlueSerial_SendByte(uint8_t Byte)
{
    DL_UART_transmitDataBlocking(Blue_Serial_INST, Byte);
}

/* ======== 数组发送 ======== */
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++)
    {
        BlueSerial_SendByte(Array[i]);
    }
}

/* ======== 字符串发送 ======== */
void BlueSerial_SendString(char *String)
{
    while (*String != '\0')
    {
        BlueSerial_SendByte((uint8_t)*String);
        String++;
    }
}

/* ======== 数字发送（指定位数） ======== */
static uint32_t BlueSerial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

void BlueSerial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        BlueSerial_SendByte((uint8_t)(Number / BlueSerial_Pow(10, Length - i - 1) % 10 + '0'));
    }
}

/* ======== 格式化打印（最大100字节） ======== */
void BlueSerial_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsnprintf(String, sizeof(String), format, arg);
    va_end(arg);
    BlueSerial_SendString(String);
}

/* ======== UART3 中断服务函数（'[' / ']' 帧解析） ======== */
void Blue_Serial_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(Blue_Serial_INST))
    {
        case DL_UART_IIDX_RX:
        {
            uint8_t RxData = DL_UART_receiveData(Blue_Serial_INST);

            static uint8_t RxState = 0;     /* 0=等待帧头, 1=接收数据 */
            static uint8_t pRxPacket = 0;   /* 当前写入位置 */

            if (RxState == 0)
            {
                if (RxData == '[' && BlueSerial_RxFlag == 0)
                {
                    RxState = 1;
                    pRxPacket = 0;
                }
            }
            else if (RxState == 1)
            {
                if (RxData == ']')
                {
                    RxState = 0;
                    BlueSerial_RxPacket[pRxPacket] = '\0';
                    BlueSerial_RxFlag = 1;
                }
                else
                {
                    if (pRxPacket < BLUE_SERIAL_RX_PACKET_MAX_LEN - 1)
                    {
                        BlueSerial_RxPacket[pRxPacket] = RxData;
                        pRxPacket++;
                    }
                    else
                    {
                        /* 超长，丢弃本帧 */
                        RxState = 0;
                        pRxPacket = 0;
                    }
                }
            }

            break;
        }

        default:
            break;
    }
}
