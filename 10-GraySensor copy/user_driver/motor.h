#ifndef __MOTOR__H
#define __MOTOR__H

#include "ti_msp_dl_config.h"

#define PI 3.14

// 编码器线数
#define MOTOR_BIANMAQI 260
// 轮胎直径 mm
#define MOTOR_WHEEL_D 48

// 50 ms测速周期内允许的最大编码器计数，用于过滤明显的干扰脉冲
#define MAX_ENCODER_COUNT 1000



void Motor_Init(void);
void Motor_SetDuty(uint8_t motor, int32_t duty);

#endif
 
