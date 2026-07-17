#include "ADC.h"
#include "string.h"

uint16_t ADC_VALUE[40];

/**
 * @brief 读取ADC数据(DMA持续搬运 + 超时轮询 → 均值滤波)
 * @param number 采样次数(≤40)
 * @return 均值滤波后的ADC值
 *
 * 前提: main()中已启动DMA和ADC(REPEAT模式), 两者持续运行
 * 本函数只清零缓冲后轮询等待DMA写入新数据, 不操作DMA/ADC寄存器
 */
unsigned int adc_getValue(unsigned int number)
{
    unsigned int gAdcResult = 0;
    volatile uint32_t timeout;

    // 清零缓冲(DMA正在后台搬运上一通道数据, 清零后等新数据覆盖)
    memset((uint16_t*)ADC_VALUE, 0, sizeof(ADC_VALUE));

    // 超时轮询: 等待DMA把 number 个新数据写入ADC_VALUE
    // 不依赖__WFI(), 纯CPU轮询 ≈ 3ms超时@80MHz
    timeout = 300000;
    while (ADC_VALUE[number - 1] == 0 && --timeout) {
        /* 空循环轮询 */
    }

    // 均值滤波
    for (unsigned char i = 0; i < number; i++) {
        gAdcResult += ADC_VALUE[i];
    }
    gAdcResult /= number;

    return gAdcResult;
}