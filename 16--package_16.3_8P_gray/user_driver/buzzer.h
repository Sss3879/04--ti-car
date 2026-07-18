#ifndef __BUZZER_H
#define __BUZZER_H


#include "ti_msp_dl_config.h"


/*
 * 蜂鸣器初始化
 */
void Buzzer_Init(void);


/*
 * 蜂鸣器开启
 */
void Buzzer_ON(void);


/*
 * 蜂鸣器关闭
 */
void Buzzer_OFF(void);


/*
 * 设置蜂鸣器频率
 *
 * freq: Hz
 */
void Buzzer_SetFreq(uint32_t freq);


/*
 * 设置占空比
 *
 * duty: 0~100 %
 */
void Buzzer_SetDuty(uint8_t duty);


#endif