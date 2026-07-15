/*              OLED      接线   SDA  PB2 |SCl  PB3 */
/*              Serial USART0      接线   PA28|PA31*/


#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include "Serial.h"
#include "stepper_motor.h"

int16_t x_error;
int16_t y_error;

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
    Serial_Printf("---  Start ---\r\n");

    // //步进电机云台-
    // Stepper_Motor_Init();
    // Stepper_Motor_Dir_Set(0,1);
    // Stepper_Motor_Start(1);
    while(1)
    {
            Serial_Printf("---  Start ---\r\n");
            delay_ms(300);
        // Stepper_Motor_Set_AngleSpeed(30,1); //对应的是多少角度每秒
        // Stepper_Motor_Set_Angle(60,1);
        // delay_ms(3000);
        // Stepper_Motor_Dir_Set(1,1);
        // delay_ms(3000);
        //     if (Serial_GetDataFlag() == 1)
        // {
        //     // x_error = Serial_GetXError();
        //     // y_error = Serial_GetYError();
        //     // Serial_Printf("Hello K230\r\n");

        //     // Serial_Printf("x:%d y:%d\r\n", x_error, y_error);
        // }
            if (Serial_RxFlag == 1)
        {

            // Serial_Printf("RX:%s\r\n", Serial_RxPacket);
            // Serial_Printf("Hello K230\r\n");
            x_error = Serial_GetXError();
            y_error = Serial_GetYError();
            
            Serial_Printf("x:%d y:%d\r\n", x_error, y_error);
            Serial_RxFlag = 0;
        }

    }

    

}
