#ifndef __BLUE_SERIAL_H
#define __BLUE_SERIAL_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdio.h>

/* 蓝牙数据包最大长度 */
#define BLUE_SERIAL_RX_PACKET_MAX_LEN  100

/* 全局：接收完成标志和数据包 */
extern char BlueSerial_RxPacket[BLUE_SERIAL_RX_PACKET_MAX_LEN];
extern volatile uint8_t BlueSerial_RxFlag;

/* 初始化蓝牙串口 (UART3, 9600, 使能RX中断) */
void BlueSerial_Init(void);

/* 发送接口 */
void BlueSerial_SendByte(uint8_t Byte);
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length);
void BlueSerial_SendString(char *String);
void BlueSerial_SendNumber(uint32_t Number, uint8_t Length);
void BlueSerial_Printf(char *format, ...);

#endif /* __BLUE_SERIAL_H */
