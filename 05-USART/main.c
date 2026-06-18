/*              OLED      接线   SDA  PB2 |SCl  PB3 */
/*              Serial USART0      接线   PA28|PA31*/


#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include "Serial.h"

int main(void)
{
    SYSCFG_DL_init();
//OLED
    // OLED_Init();
    // OLED_Clear();
    // uint32_t a=20;
    // char oled_str[40];
    // sprintf(oled_str, "a = %d", a);
    // while (1) {
    //     OLED_ShowString(0, 0,(u8*) "hello world", 12);
    //     OLED_ShowString(0, 13,(u8*) oled_str, 12);
    //     OLED_Refresh();
    // }
//串口
    NVIC_EnableIRQ(Serial_INST_INT_IRQN);
    while(1)
    {
        delay_ms(1000);
        UART_send_string(Serial_INST, "hello");
    }

    

}
