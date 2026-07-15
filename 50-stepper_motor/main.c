/*              OLED      接线   SDA  PB2 |SCl  PB3 */
/*              Serial USART0      接线   PA28|PA31*/


#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include "Serial.h"
#include "stepper_motor.h"

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

    //步进电机云台-
    Stepper_Motor_Init();
    Stepper_Motor_Start(1);
    Stepper_Motor_Dir_Set(0,1);

    // Stepper_Motor_Start(2);
    // Stepper_Motor_Dir_Set(0,2);


    
    
    
    while(1)
    {
        // delay_ms(1000);
        // Serial_Printf("---  Start ---\r\n");

        //一脉冲0.05625度
        //角速度= 0.05625度*脉冲频率
        //脉冲频率=角速度/0.05625度
        // 角速度:30 / 0.05625 = 533.33Hz

//         Stepper_Motor_Step_Set(1,2);
//         delay_ms(5);
//         Stepper_Motor_Step_Set(0,2);
//         delay_ms(5);

//         // Stepper_Motor_Set_AngleSpeed(30,1); //对应的是多少角度每秒

        // Stepper_Motor_Dir_Set(1, 2);
        // delay_ms(2000);
        // Stepper_Motor_Dir_Set(0, 2);
        // Stepper_Motor_Set_AngleSpeed(10, 2);
        // delay_ms(3000);
        // Stepper_Motor_Stop(2);

//         // Stepper_Motor_Set_Angle(60,1);
//         // delay_ms(3000);
//         // Stepper_Motor_Dir_Set(1,1);
//         // delay_ms(3000);



//测试配置是否成功，配置是否成功接线，一定要延时！！！
        Stepper_Motor_Set_AngleSpeed(30,2); //对应的是多少角度每秒

        Stepper_Motor_Set_Angle(60,2);
        delay_ms(3000);
        Stepper_Motor_Dir_Set(1,2);
        Stepper_Motor_Set_Angle(60,2);
        delay_ms(3000);


        Stepper_Motor_Set_AngleSpeed(30,1); //对应的是多少角度每秒

        Stepper_Motor_Set_Angle(60,1);
        delay_ms(3000);
        Stepper_Motor_Dir_Set(1,1);
        Stepper_Motor_Set_Angle(60,1);
        delay_ms(3000);
    }

    

}
