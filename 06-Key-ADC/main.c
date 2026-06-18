/*              OLED      接线   SDA  PB2 |SCl  PB3 */
/*              Serial USART0      接线   PA28|PA31*/
/*              Key按键                  PB6|PB7  */
/*              POT电位器旋钮             PA16       */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include "Serial.h"
volatile  uint8_t status=0;
int main(void)
{
    SYSCFG_DL_init();
//OLED
    OLED_Init();
    OLED_Clear();
    // uint32_t a=20;
    // char oled_str[40];
    // sprintf(oled_str, "a = %d", a);
    // while (1) {
    //     OLED_ShowString(0, 0,(u8*) "hello world", 12);
    //     OLED_ShowString(0, 13,(u8*) oled_str, 12);
    //     OLED_Refresh();
    // }
// //串口
//     NVIC_EnableIRQ(Serial_INST_INT_IRQN);
    
//     while(1)
//     {
//         delay_ms(1000);
//         UART_send_string(Serial_INST, "hello");
//     }


//Key


    NVIC_EnableIRQ(Serial_INST_INT_IRQN);
    NVIC_EnableIRQ(Key_INT_IRQN);  

    DL_ADC12_enableConversions(POT_INST);

    char buffer[64];
    while(1)
    {
        DL_ADC12_startConversion(POT_INST);
        delay_ms(100);
        uint16_t adc_value = DL_ADC12_getMemResult(POT_INST,POT_ADCMEM_0);
        snprintf(buffer, sizeof(buffer), "ADC Value: %u\r\n", adc_value);
        UART_send_string(Serial_INST, buffer);
        OLED_ShowString(0, 0,(u8*) buffer, 16);
        OLED_Refresh();

        if(status==1)
        {
            snprintf(buffer, sizeof(buffer), "Key1 Pressed\r\n");
            UART_send_string(Serial_INST, buffer);
            status=0;
        }
        else if(status==2)
        {

            snprintf(buffer, sizeof(buffer), "Key2 Pressed\r\n");
            UART_send_string(Serial_INST, buffer);
            status=0;
        }
    }
    

}

void GROUP1_IRQHandler()
{
    
    switch (DL_GPIO_getPendingInterrupt(Key_PORT))
    {
    case Key_Key1_IIDX:
        /* code */
        status = (status + 1) % 3;
        break;
    case Key_Key2_IIDX:
        status = (status + 3 -1) % 3;
        /* code */
        break;
    
    default:
        break;
    }
}
