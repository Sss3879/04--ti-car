/*              OLED      接线   SDA  PB3 |SCl  PB2 */
/*              Serial USART0      接线   PA28|PA31*/
/*              Key按键                  PB6|PB7  */
/*              POT电位器旋钮             PA16       */
/*              POT电位器旋钮             PA16       */
#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include "Serial.h"
#include "motor.h"
#include "PID.h"
#include "uart_vofa.h"
volatile  uint8_t status=0;
extern float speed_1;
extern float speed_2;
char buffer[64];
// Motor_Left PID:
//   Target(左轮目标) + Actual(speed_1=左编码器) → Out → Motor_SetDuty(2, ...) → motor 2 → 物理左电机 ✓

// Motor_Right PID:
//   Target(右轮目标) + Actual(speed_2=右编码器) → Out → Motor_SetDuty(1, ...) → motor 1 → 物理右电机 ✓

PID_t Motor_Left={
	
	.Kp=2.64f,
	.Ki=1.47f,
	.Kd=0,
	.Target=100,
	.OutMax=400,
	.OutMin=-400,
	
};

PID_t Motor_Right={
	
	.Kp=1.85f,
	.Ki=1.3f,
	.Kd=0,
	.Target=100,
	.OutMax=400,
	.OutMin=-400,
	
};

int main(void)
{
    SYSCFG_DL_init();

// //串口
//     NVIC_EnableIRQ(Serial_INST_INT_IRQN);
    
//     while(1)
//     {
//         delay_ms(1000);
//         UART_send_string(Serial_INST, "hello");
//     }

//Motor
   
    
    NVIC_EnableIRQ(Serial_INST_INT_IRQN);

    NVIC_EnableIRQ(Motor_GPIOA_INT_IRQN);          // E1A = PA21
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN); // E2A + Key1 + Key2

    Motor_Init();
    while(1)
    {

        // Motor_SetDuty(1,1000);
        // Motor_SetDuty(2,1000);
        
        // snprintf(buffer, sizeof(buffer), " speed_1: %f\r\n", speed_1);
        // UART_send_string(Serial_INST, buffer);
        // snprintf(buffer, sizeof(buffer), " speed_2: %f\r\n", speed_2);
        // UART_send_string(Serial_INST, buffer);
        // delay_ms(1000);

        vofa_draw_graphical(2);
        if (Serial_RxFlag == 1)
            {
                vofa_parse_packet(Serial_RxPacket);
                // Serial_Printf("RX:%s\r\n", Serial_RxPacket);
                Serial_RxFlag = 0;
            }


    }

    
}
volatile int32_t Encoder1_Count = 0;
volatile int32_t Encoder2_Count = 0;



void GROUP1_IRQHandler(void)
{
    uint32_t gpioa_pending;
    uint32_t gpiob_pending;

    gpioa_pending = DL_GPIO_getPendingInterrupt(GPIOA);
    gpiob_pending = DL_GPIO_getPendingInterrupt(GPIOB);

    // 编码器1：E1A = PA21，E1B = PA22
    if(gpioa_pending == Motor_E1A_IIDX)
    {
        if(DL_GPIO_readPins(Motor_E1B_PORT, Motor_E1B_PIN))
        {
            Encoder1_Count++;
        }
        else
        {
            Encoder1_Count--;
        }
    }

    // 编码器2 + 按键：都在 GPIOB
    switch(gpiob_pending)
    {
        case Motor_E2A_IIDX:
            if(DL_GPIO_readPins(Motor_E2B_PORT, Motor_E2B_PIN))
            {
                Encoder2_Count++;
            }
            else
            {
                Encoder2_Count--;
            }
            break;

        case Key_Key1_IIDX:
            status = (status + 1) % 3;
            break;

        case Key_Key2_IIDX:
            status = (status + 3 - 1) % 3;
            break;

        default:
            break;
    }
}