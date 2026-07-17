#ifndef NO_MCU_GANV_GRAYSCALE_SENSOR_CONFIG_H_
#define NO_MCU_GANV_GRAYSCALE_SENSOR_CONFIG_H_

#include <string.h>
#include "ti_msp_dl_config.h"
#include "ADC.h"
#include "delay.h"

/**************************** 传感器版本配置 ****************************/
#define Class           0           // 基础版

/**************************** ADC分辨率配置 ****************************/
#define _14Bits 0
#define _12Bits 1
#define _10Bits 2
#define _8Bits  3

/**************************** 用户可配置区域 ***************************/
// 传感器版本选择
#define Sensor_Edition      Class

// 输出结果方向, 与预期方向不同选1
#define Direction           1

// ADC分辨率选择
// #define Sensor_ADCbits  _14Bits
#define Sensor_ADCbits      _12Bits   // MSPM0G3507 12位ADC
// #define Sensor_ADCbits  _10Bits
// #define Sensor_ADCbits  _8Bits

/*************************** 硬件抽象层: 地址引脚定义 ****************************/
// 地址线0: PB8
#define ADDR0_PORT          GPIOB
#define ADDR0_PIN           DL_GPIO_PIN_11

// 地址线1: PB9
#define ADDR1_PORT          GPIOB
#define ADDR1_PIN           DL_GPIO_PIN_12

// 地址线2: PA17
#define ADDR2_PORT          GPIOA
#define ADDR2_PIN           DL_GPIO_PIN_17

// GPIO地址切换宏
#define Switch_Address_0(i) ((i) ? (DL_GPIO_setPins(ADDR0_PORT, ADDR0_PIN)) : (DL_GPIO_clearPins(ADDR0_PORT, ADDR0_PIN)))
#define Switch_Address_1(i) ((i) ? (DL_GPIO_setPins(ADDR1_PORT, ADDR1_PIN)) : (DL_GPIO_clearPins(ADDR1_PORT, ADDR1_PIN)))
#define Switch_Address_2(i) ((i) ? (DL_GPIO_setPins(ADDR2_PORT, ADDR2_PIN)) : (DL_GPIO_clearPins(ADDR2_PORT, ADDR2_PIN)))
/**********************************************************************/

/*************************** 传感器数据结构 ***************************/
typedef struct {
    unsigned short Analog_value[8];      // 原始模拟量值
    unsigned short Normal_value[8];      // 归一化后的值
    unsigned short Calibrated_white[8];  // 白校准基准值
    unsigned short Calibrated_black[8];  // 黑校准基准值
    unsigned short Gray_white[8];        // 白平衡灰度值
    unsigned short Gray_black[8];        // 黑平衡灰度值
    double Normal_factor[8];             // 归一化系数
    double bits;                         // ADC分辨率对应位数
    unsigned char Digtal;                // 数字输出状态
    unsigned char Time_out;              // 超时标志
    unsigned char Tick;                  // 时基计数器
    unsigned char ok;                    // 传感器就绪标志
} No_MCU_Sensor;

#ifdef __cplusplus
extern "C" {
#endif

/*************************** 函数声明 *****************************/
void No_MCU_Ganv_Sensor_Init_Frist(No_MCU_Sensor* sensor);
void No_MCU_Ganv_Sensor_Init(No_MCU_Sensor* sensor, unsigned short* Calibrated_white, unsigned short* Calibrated_black);
void No_Mcu_Ganv_Sensor_Task_Without_tick(No_MCU_Sensor* sensor);
unsigned char Get_Digtal_For_User(No_MCU_Sensor* sensor);
unsigned char Get_Normalize_For_User(No_MCU_Sensor* sensor, unsigned short* result);
unsigned char Get_Anolog_Value(No_MCU_Sensor* sensor, unsigned short* result);
										 
int16_t GraySensor_GetLineError(No_MCU_Sensor *sensor);
uint8_t GraySensor_GetLostFlag(void);			

#ifdef __cplusplus
}
#endif

#endif /* NO_MCU_GANV_GRAYSCALE_SENSOR_CONFIG_H_ */
