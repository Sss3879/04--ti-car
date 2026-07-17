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
#include "BlueSerial.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
volatile  uint8_t status=0;
extern float speed_1;
extern float speed_2;
char buffer[64];

// Motor_Left PID:
//   Target(左轮目标) + Actual(speed_1=左编码器) → Out → Motor_SetDuty(2, ...) → motor 2 → 物理左电机 ✓

// Motor_Right PID:
//   Target(右轮目标) + Actual(speed_2=右编码器) → Out → Motor_SetDuty(1, ...) → motor 1 → 物理右电机 ✓

PID_t Motor_Left={

	.Kp=4.56f,
	.Ki=1.01f,
	.Kd=0,
	.Target=0,
	.OutMax=800,
	.OutMin=-800,

};

PID_t Motor_Right={

	.Kp=3.97f,
	.Ki=0.95f,
	.Kd=0,
	.Target=0,
	.OutMax=800,
	.OutMin=-800,

};

// ======== 灰度传感器变量 ========
	// 黑白校准值(实测标定: 黑≈330, 白≈1600~3000)
	unsigned short white[8] = {871,874,1437,1436,1444,1439,1611,1617   };
	unsigned short black[8] = {237,235,245,250,250,248,250,252  };
unsigned short Anolog[8]  = {0};
unsigned short Normal[8]  = {0};

No_MCU_Sensor sensor;
unsigned char Digtal;


int16_t Line_Error = 0;
uint8_t Line_LostFlag = 0;

int16_t BaseSpeed = 120;
float Line_Kp = 0.03f;


#define SPEED_MAX 180
#define SPEED_MIN -180

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

    if (Turn > 50)
    {
        Turn = 50;
    }
    else if (Turn < -50)
    {
        Turn = -50;
    }

    Motor_Left.Target  = BaseSpeed + Turn;
    Motor_Right.Target = BaseSpeed - Turn;

    // 限制目标速度不要超过 ±30
    Motor_Left.Target  = Limit_Float(Motor_Left.Target, SPEED_MIN, SPEED_MAX);
    Motor_Right.Target = Limit_Float(Motor_Right.Target, SPEED_MIN, SPEED_MAX);
}



int main(void)
{
    SYSCFG_DL_init();

    // ---- 灰度传感器初始化 ----
    No_MCU_Ganv_Sensor_Init(&sensor, white, black);

    // 配置DMA源/目的地址并启动（REPEAT模式持续搬运, adc_getValue只轮询读）
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&ADC0->ULLMEM.MEMRES[0]);
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&ADC_VALUE[0]);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_ADC12_startConversion(ADC12_0_INST);  // 软件触发ADC, REPEAT模式自动连续

    NVIC_EnableIRQ(Serial_INST_INT_IRQN);

    NVIC_EnableIRQ(Motor_GPIOA_INT_IRQN);          // E1A = PA21
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN); // E2A + Key1 + Key2

    Motor_Init();

    BlueSerial_Init();
    NVIC_EnableIRQ(Blue_Serial_INST_INT_IRQN);

    Serial_Printf("--- GraySensor Start ---\r\n");
    BlueSerial_Printf("--- GraySensor Start ---\r\n");

    while(1)
    {
        // ======== 灰度传感器任务 ========
        No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);

        // 获取数字量（8位, 每位对应一个通道: 1=白 0=黑）
        Digtal = Get_Digtal_For_User(&sensor);

        // 获取原始模拟值
        Get_Anolog_Value(&sensor, Anolog);

        // 获取归一化值
        Get_Normalize_For_User(&sensor, Normal);

        // ======== 串口输出传感器数据 ========
        Serial_Printf("D:%d%d%d%d%d%d%d%d ",
            (Digtal>>0)&1, (Digtal>>1)&1, (Digtal>>2)&1, (Digtal>>3)&1,
            (Digtal>>4)&1, (Digtal>>5)&1, (Digtal>>6)&1, (Digtal>>7)&1);

        Serial_Printf("A:%d,%d,%d,%d,%d,%d,%d,%d ",
            Anolog[0], Anolog[1], Anolog[2], Anolog[3],
            Anolog[4], Anolog[5], Anolog[6], Anolog[7]);

        Serial_Printf("N:%d,%d,%d,%d,%d,%d,%d,%d\r\n",
            Normal[0], Normal[1], Normal[2], Normal[3],
            Normal[4], Normal[5], Normal[6], Normal[7]);

        // ======== 蓝牙串口输出 ========
        BlueSerial_Printf("[Gray D:%d%d%d%d%d%d%d%d A:%d,%d,%d,%d,%d,%d,%d,%d]\r\n",
            (Digtal>>0)&1, (Digtal>>1)&1, (Digtal>>2)&1, (Digtal>>3)&1,
            (Digtal>>4)&1, (Digtal>>5)&1, (Digtal>>6)&1, (Digtal>>7)&1,
            Anolog[0], Anolog[1], Anolog[2], Anolog[3],
            Anolog[4], Anolog[5], Anolog[6], Anolog[7]);

        //灰度巡线任务
        Car_LineControl();

        Serial_Printf("%d,%d,%.2f,%.2f,%.2f,%.2f\r\n",
                        Line_Error,
                        Line_LostFlag,
                        Motor_Left.Target,
                        Motor_Right.Target,
                        Motor_Left.Actual,
                        Motor_Right.Actual);
    

        // ======== VOFA+ PID调试（可注释上面Serial_Printf，改用这个看波形） ========
        // vofa_draw_graphical(2);

        // ======== 串口命令接收 ========
        if (Serial_RxFlag == 1)
        {
            vofa_parse_packet(Serial_RxPacket);
            Serial_RxFlag = 0;
        }

        // ======== 循迹控制（根据实际需求修改）========
        // if (!(Digtal & 0x08)) { /* 中间通道4检测到黑线 */ }

        delay_ms(1);
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
        /* 只在A相有效边沿判向，避免抖动造成错误计数。 */
        if (DL_GPIO_readPins(Motor_E1A_PORT, Motor_E1A_PIN))
        {
            delay_cycles(100);
            if (DL_GPIO_readPins(Motor_E1A_PORT, Motor_E1A_PIN))
            {
                if(DL_GPIO_readPins(Motor_E1B_PORT, Motor_E1B_PIN))
                {
                    Encoder1_Count--;
                }
                else
                {
                    Encoder1_Count++;
                }
            }
        }
    }

    // 编码器2 + 按键：都在 GPIOB
    switch(gpiob_pending)
    {
        case Motor_E2A_IIDX:
            if (!DL_GPIO_readPins(Motor_E2A_PORT, Motor_E2A_PIN)) break;
            delay_cycles(100);
            if (!DL_GPIO_readPins(Motor_E2A_PORT, Motor_E2A_PIN)) break;

            if(DL_GPIO_readPins(Motor_E2B_PORT, Motor_E2B_PIN))
            {
                Encoder2_Count--;
            }
            else
            {
                Encoder2_Count++;
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
