/*              OLED      接线   SDA  PB3 |SCl  PB2 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>


int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    // OLED_ColorTurn(0);
    OLED_Clear();
    uint32_t a=20;
    char oled_str[40];
    sprintf(oled_str, "a = %d", a);
            OLED_ShowString(0, 0,(u8*) "hello world", 12);
        OLED_ShowString(0, 13,(u8*) oled_str, 12);
        OLED_Refresh();
    while (1) {
        // OLED_ShowString(0, 0,(u8*) "hello world", 12);
        // OLED_ShowString(0, 13,(u8*) oled_str, 12);
        // OLED_Refresh();
    }
}
