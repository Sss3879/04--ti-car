/* ========================================================================
 *                              06-2key
 *                     MSPM0G3507 双按键检测
 * ========================================================================
 *  接线说明：
 *    OLED    SDA=PB3  SCL=PB2
 *    Serial  TX=PA28  RX=PA31  (115200)
 *    Key1    PA24  (下降沿中断, 内部下拉)
 *    Key2    PA23  (下降沿中断, 内部下拉)
 * ======================================================================== */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include "Serial.h"

volatile uint8_t status = 0;

int main(void)
{
    SYSCFG_DL_init();

    /* ---- OLED ---- */
    OLED_Init();
    OLED_Clear();

    /* ---- 使能中断 ---- */
    NVIC_EnableIRQ(Serial_INST_INT_IRQN);
    NVIC_EnableIRQ(Key_INT_IRQN);

    /* ---- 启动 ---- */
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Key Test Start\r\n");
    UART_send_string(Serial_INST, buffer);

    while (1)
    {
        if (status == 1)
        {
            snprintf(buffer, sizeof(buffer), "Key1 Pressed\r\n");
            UART_send_string(Serial_INST, buffer);
            status = 0;
        }
        else if (status == 2)
        {
            snprintf(buffer, sizeof(buffer), "Key2 Pressed\r\n");
            UART_send_string(Serial_INST, buffer);
            status = 0;
        }
    }
}

/* ---- GPIO 中断：Key1(PA24) + Key2(PA23) ---- */
void GROUP1_IRQHandler(void)
{
    switch (DL_GPIO_getPendingInterrupt(Key_PORT))
    {
        case Key_Key1_IIDX:
            status = (status + 1) % 3;   /* 按下Key1: 切换到状态1 */
            break;
        case Key_Key2_IIDX:
            status = (status + 2) % 3;   /* 按下Key2: 切换到状态2 */
            break;
        default:
            break;
    }
}
