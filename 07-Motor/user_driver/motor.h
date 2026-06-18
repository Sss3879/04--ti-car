#ifndef __MOTOR__H
#define __MOTOR__H

#include "ti_msp_dl_config.h"

#define PI 3.14
//编码器线数字
#define Motor_Encoder 260
//轮胎直径mm
#define Motor_Wheel_Diameter 48

void Motor_Init(void);
void Motor_SetDuty(uint8_t motor, int32_t duty);

#endif
 