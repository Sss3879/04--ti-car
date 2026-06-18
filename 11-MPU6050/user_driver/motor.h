#ifndef __MOTOR__H
#define __MOTOR__H

#include "ti_msp_dl_config.h"

#define PI 3.14

// 编码器线数
#define MOTOR_BIANMAQI 260
// 轮胎直径 mm
#define MOTOR_WHEEL_D 48

// 50ms内最大合理脉冲数（260线编码器 300RPM≈65脉冲/50ms，取200留余量）
#define MAX_ENCODER_COUNT 200



void Motor_Init(void);
void Motor_SetDuty(uint8_t motor, int32_t duty);

#endif
 