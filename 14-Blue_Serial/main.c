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

    BlueSerial_Init();          // 使能蓝牙 UART3 RX 中断

    // // // ---- 灰度传感器初始化 ----
    // // No_MCU_Ganv_Sensor_Init(&sensor, white, black);

    // // 配置DMA源/目的地址并启动（REPEAT模式持续搬运, adc_getValue只轮询读）
    // DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&ADC0->ULLMEM.MEMRES[0]);
    // DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&ADC_VALUE[0]);
    // DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
    // DL_ADC12_startConversion(ADC12_0_INST);  // 软件触发ADC, REPEAT模式自动连续

    // NVIC_EnableIRQ(Serial_INST_INT_IRQN);

    // NVIC_EnableIRQ(Motor_GPIOA_INT_IRQN);          // E1A = PA21
    // NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN); // E2A + Key1 + Key2

    // Motor_Init();
	IMU_init();

    TimeA1_Init();


    Serial_Printf("---  Start ---\r\n");

    delay_ms(20);
    


    while (1)
    {

        if(ICM42688_UpdateFlag==1)
        {
            ICM42688_UpdateFlag=0;
            Angle_PID.Actual = ypr[0];
			PID_Position(&Angle_PID);
			Motor_Right.Target  = BaseSpeed - Angle_PID.Out;
			Motor_Left.Target = BaseSpeed + Angle_PID.Out;			
            
            // vofa_draw_graphical(3);
            BlueSerial_Printf("[plot,%.2f,%.2f,%.2f,%.2f,%.2f]",
                              Angle_PID.Target,
                              Angle_PID.Actual,
                              Angle_PID.Out,
                              Motor_Left.Target,
                              Motor_Right.Target);

        }          


        // ======== 串口命令接收 ========
        if (Serial_RxFlag == 1)
        {
            vofa_parse_packet(Serial_RxPacket);
            Serial_RxFlag = 0;
        }
        
        // if(printf_flag==1)
        // {
        //     printf_flag=0;
        //     vofa_draw_graphical(2);
        // }
        // ======== 蓝牙串口命令接收 ========
        if (BlueSerial_RxFlag == 1)
        {
            char *Tag = strtok(BlueSerial_RxPacket, ",");

            if (Tag != NULL)
            {
                // --- 按键命令：key,Stop,down ---
                if (strcmp(Tag, "key") == 0)
                {
                    char *Name = strtok(NULL, ",");
                    char *Action = strtok(NULL, ",");

                    if (Name != NULL && Action != NULL)
                    {
                        if (strcmp(Name, "Stop") == 0 && strcmp(Action, "down") == 0)
                        {
                            BaseSpeed = 0;
                            Motor_Left.Target = 0;
                            Motor_Right.Target = 0;
                        }
                    }
                }

                // --- 滑块命令：slider,参数名,参数值 ---
                else if (strcmp(Tag, "slider") == 0)
                {
                    char *Name = strtok(NULL, ",");
                    char *Value = strtok(NULL, ",");

                    if (Name != NULL && Value != NULL)
                    {
                        if (strcmp(Name, "AngleKp") == 0)
                            Angle_PID.Kp = atof(Value);
                        else if (strcmp(Name, "AngleKi") == 0)
                            Angle_PID.Ki = atof(Value);
                        else if (strcmp(Name, "AngleKd") == 0)
                            Angle_PID.Kd = atof(Value);
                        else if (strcmp(Name, "BaseSpeed") == 0)
                            BaseSpeed = atof(Value);
                        else if (strcmp(Name, "LeftKp") == 0)
                            Motor_Left.Kp = atof(Value);
                        else if (strcmp(Name, "LeftKi") == 0)
                            Motor_Left.Ki = atof(Value);
                        else if (strcmp(Name, "LeftKd") == 0)
                            Motor_Left.Kd = atof(Value);
                        else if (strcmp(Name, "RightKp") == 0)
                            Motor_Right.Kp = atof(Value);
                        else if (strcmp(Name, "RightKi") == 0)
                            Motor_Right.Ki = atof(Value);
                        else if (strcmp(Name, "RightKd") == 0)
                            Motor_Right.Kd = atof(Value);
                    }
                }

                // --- 摇杆命令：joystick,LH,LV,RH,RV ---
                else if (strcmp(Tag, "joystick") == 0)
                {
                    char *LH_Str = strtok(NULL, ",");
                    char *LV_Str = strtok(NULL, ",");
                    char *RH_Str = strtok(NULL, ",");
                    char *RV_Str = strtok(NULL, ",");

                    if (LH_Str != NULL && LV_Str != NULL
                        && RH_Str != NULL && RV_Str != NULL)
                    {
                        int8_t LH = atoi(LH_Str);
                        int8_t LV = atoi(LV_Str);
                        int8_t RH = atoi(RH_Str);
                        int8_t RV = atoi(RV_Str);

                        // LV 控制基准速度（摇杆范围 -100 ~ 100）
                        BaseSpeed = LV / 5.0f;

                        // RH 控制目标角度（转向）
                        Angle_PID.Target = RH / 5.0f;

                        // LH, RV 暂未使用
                        (void)LH;
                        (void)RV;
                    }
                }
            }

        BlueSerial_RxFlag = 0;
    }
     }
        // printf("%.2f,%.2f,%.2f\r\n",ypr[0],ypr[1],ypr[2]);
        // delay_ms(100);




}

    


void TIMER_0_INST_IRQHandler(void)
{
	switch(DL_TimerG_getPendingInterrupt(TIMER_0_INST))
	{
		case DL_TIMER_IIDX_ZERO:
		    IMU_getYawPitchRoll(ypr);
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
