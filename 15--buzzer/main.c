/*              OLED      接线   SDA  PB3 |SCl  PB2 */
/*              Serial USART0      接线   PA28|PA31*/
/*              Key按键                  PB6|PB7  */
/*              POT电位器旋钮             PA16       */


#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include "Serial.h"
#include "motor.h"
#include "PID.h"
#include "uart_vofa.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "icm42688.h"
#include "IMU.h"
#include "BlueSerial.h"
#include <string.h>   // strtok, strcmp 需要
#include <stdlib.h>   // atof, atoi 需要
#include "buzzer.h"

volatile  uint8_t status=0;
extern float speed_1;
extern float speed_2;
char buffer[64];
extern uint8_t printf_flag ;
extern int32_t dbg_c1 , dbg_c2 ;
PID_t Motor_Left={

	.Kp=1.00f,
	.Ki=1.77f,
	.Kd=0,
	.Target=50,
	.OutMax=500, 
	.OutMin=-500,

};

PID_t Motor_Right={

	.Kp=1.27f,
	.Ki=2.00f,
	.Kd=0,
	.Target=50,
	.OutMax=500,
	.OutMin=-500,

};

PID_t Angle_PID={

	.Kp=2.17,
	.Ki=0,
	.Kd=0.28,
	.Target=0,   //保持直走
	.OutMax=90,
	.OutMin=-90,

};

// ======== 灰度传感器变量 ========
	// 黑白校准值(实测标定: 黑≈330, 白≈1600~3000)
	unsigned short white[8] = {520, 2900, 2500, 2000, 1550, 1850, 2100, 1650};
	unsigned short black[8] = {350,  350,  350,  350,  350,  350,  350,  350};
unsigned short Anolog[8]  = {0};
unsigned short Normal[8]  = {0};

No_MCU_Sensor sensor;
unsigned char Digtal;


int16_t Line_Error = 0;
uint8_t Line_LostFlag = 0;

int16_t BaseSpeed =50;
float Line_Kp = 0.03f;


#define SPEED_MAX 40
#define SPEED_MIN -40

float Limit_Float(float value, float min, float max)
{
    if (value > max)
    {
        return max;
    }
    else if (value < min)
    {
        return min;
    }
    return value;
}
void Car_LineControl(void)
{
    int16_t Turn;

    Line_Error = GraySensor_GetLineError(&sensor);
    Line_LostFlag = GraySensor_GetLostFlag();

    Turn = (int16_t)(Line_Kp * Line_Error);

    if (Turn > 15)
    {
        Turn = 15;
    }
    else if (Turn < -15)
    {
        Turn = -15;
    }

    Motor_Left.Target  = BaseSpeed + Turn;
    Motor_Right.Target = BaseSpeed - Turn;

    // 限制目标速度不要超过 ±30
    Motor_Left.Target  = Limit_Float(Motor_Left.Target, SPEED_MIN, SPEED_MAX);
    Motor_Right.Target = Limit_Float(Motor_Right.Target, SPEED_MIN, SPEED_MAX);
}


//---------------------------ICM42688 -------------------------
//--------------------------          -----------------------


//ICM42688接线示例
// VCC--------5V或者3.3V都可以
//IIC 模式接线
// PA0------------------------SDA
// PA1------------------------SCL

void delay_ms(uint32_t __ms);
float ypr[3];          // 上传yaw pitch roll的值

uint8_t ICM42688_UpdateFlag=0;
int fputc(int ch, FILE *stream)
{
    while( DL_UART_isBusy(Serial_INST) == true );
    DL_UART_Main_transmitData(Serial_INST, ch);
    return ch;
}

int fputs(const char* restrict s, FILE* restrict stream) {
    uint16_t char_len=0;
    while(*s!=0)
    {
        while( DL_UART_isBusy(Serial_INST) == true );
        DL_UART_Main_transmitData(Serial_INST, *s++);
        char_len++;
    }
    return char_len;
}

int puts(const char* _ptr)
{
 return 0;
}

void TimeA1_Init(void)
{
	NVIC_ClearPendingIRQ(TIMG6_INT_IRQn);
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}


int main(void)
{
    SYSCFG_DL_init();

    Buzzer_Init();

    TimeA1_Init();


    // Serial_Printf("---  Start ---\r\n");

    delay_ms(20);
    


    while (1)
    {
         //设置2kHz
        Buzzer_SetFreq(2000);


        //50%占空比
        Buzzer_SetDuty(50);


        //响
        Buzzer_ON();


    }

}

    


void TIMER_0_INST_IRQHandler(void)
{
	switch(DL_TimerG_getPendingInterrupt(TIMER_0_INST))
	{
		case DL_TIMER_IIDX_ZERO:
		    // IMU_getYawPitchRoll(ypr);
            ICM42688_UpdateFlag=1;
		    break;
		default:
			
		    break;
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
