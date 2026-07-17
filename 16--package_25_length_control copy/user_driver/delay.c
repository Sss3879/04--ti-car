#include "delay.h"

void delay_ms(uint32_t ms)
{
    uint32_t cycles=(CPUCLK_FREQ/1000)*ms;
    delay_cycles(cycles);
}

/**
 * @brief 微秒延时(粗略, 用于传感器通道切换)
 * @param us 延时时间(us), 80MHz下约 us*80 次循环
 */
void delay_us(uint32_t us)
{
    volatile uint32_t count = us * 80;
    while (count--);
}
