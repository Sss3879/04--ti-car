/* ========================================================================
 *         16--package_25_length_control-calibrate_length — 编码器标定
 *                       MSPM0G3507
 * ========================================================================
 *
 *  【用途】推车测编码器脉冲数，标定距离→脉冲的换算关系
 *
 *  【为什么需要】不同车模的轮径、编码器线数、减速比不同，
 *   必须先实测一次，才能知道"走 50cm 需要多少脉冲"。
 *
 *  【操作步骤】
 *    1. 烧录、上电（电机未初始化，轮子可自由转动）
 *    2. 按 Key1 清零编码器（OLED 和蓝牙都会提示）
 *    3. 把车放在起点，匀速推 50cm 到终点
 *    4. 看 OLED 第4行 Avg 的值（或蓝牙打印的 Avg）
 *    5. 告诉 Claude 这个数值
 *
 *  【注意】
 *    - 左右轮方向相反，E1/E2 一正一负是正常的，Avg 已取绝对值
 *    - 本工程没有初始化电机，纯测试用途
 *    - 标定完成后的数据用于 25_length_control 的 STRAIGHT_PULSE
 *
 *  【示例数据】
 *    推 50cm → Avg=886
 *    100cm 矩形边长 → STRAIGHT_PULSE = 886×2 = 1772
 *
 * ========================================================================
 *  Key1 = 清零编码器
 *  OLED 实时显示两轮编码器值和绝对值平均
 *  蓝牙 每 200ms 发送 E1/E2/Avg
 * ======================================================================== */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include "Serial.h"
#include "BlueSerial.h"
#include "PID.h"

/* motor.o 依赖的空壳（本工程不用电机，仅满足链接） */
PID_t Motor_Left  = {0};
PID_t Motor_Right = {0};

volatile uint8_t status = 0;
volatile int32_t Encoder1_Count = 0;
volatile int32_t Encoder2_Count = 0;

int main(void)
{
    SYSCFG_DL_init();
    delay_ms(100);
    BlueSerial_Init();
    delay_ms(1000);
    OLED_Init();

    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(Motor_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(Serial_INST_INT_IRQN);

    BlueSerial_Printf("Encoder Test Start\r\nPush car 50cm\r\n");

    while (1)
    {
        /* Key1 清零编码器 */
        if (status == 1) {
            Encoder1_Count = 0;
            Encoder2_Count = 0;
            BlueSerial_Printf("Encoder cleared\r\n");
            status = 0;
        }

        /* OLED 刷新（用绝对值，因左右轮方向相反） */
        {
            char line[32];
            int32_t e1 = Encoder1_Count, e2 = Encoder2_Count;
            if (e1 < 0) e1 = -e1;
            if (e2 < 0) e2 = -e2;
            int32_t avg = (e1 + e2) / 2;

            snprintf(line, sizeof(line), "Push 50cm, Key1=clr");
            OLED_ShowString(0, 0, (u8 *)line, 12);
            snprintf(line, sizeof(line), "E1:%d", (int)Encoder1_Count);
            OLED_ShowString(0, 16, (u8 *)line, 12);
            snprintf(line, sizeof(line), "E2:%d", (int)Encoder2_Count);
            OLED_ShowString(0, 32, (u8 *)line, 12);
            snprintf(line, sizeof(line), "Avg:%d", (int)avg);
            OLED_ShowString(0, 48, (u8 *)line, 12);
            OLED_Refresh();
        }

        /* 蓝牙打印（绝对值） */
        {
            int32_t e1 = Encoder1_Count, e2 = Encoder2_Count;
            if (e1 < 0) e1 = -e1;
            if (e2 < 0) e2 = -e2;
            int32_t avg = (e1 + e2) / 2;
            BlueSerial_Printf("E1:%d E2:%d Avg:%d\r\n",
                              (int)Encoder1_Count, (int)Encoder2_Count, (int)avg);
        }

        delay_ms(200);
    }
}

/* ---- GPIO 中断：编码器 + 按键 ---- */
void GROUP1_IRQHandler(void)
{
    uint32_t iidx;

    while ((iidx = DL_GPIO_getPendingInterrupt(GPIOA)) != 0)
    {
        switch (iidx)
        {
            case Motor_E1A_IIDX:
                if (DL_GPIO_readPins(Motor_E1A_PORT, Motor_E1A_PIN))
                {
                    delay_cycles(100);
                    if (DL_GPIO_readPins(Motor_E1A_PORT, Motor_E1A_PIN))
                    {
                        if (DL_GPIO_readPins(Motor_E1B_PORT, Motor_E1B_PIN))
                            Encoder1_Count--;
                        else
                            Encoder1_Count++;
                    }
                }
                break;
            case Key_Key1_IIDX:
                status = 1;
                break;
            default:
                break;
        }
    }

    while ((iidx = DL_GPIO_getPendingInterrupt(GPIOB)) != 0)
    {
        if (iidx == Motor_E2A_IIDX)
        {
            if (DL_GPIO_readPins(Motor_E2A_PORT, Motor_E2A_PIN))
            {
                delay_cycles(100);
                if (DL_GPIO_readPins(Motor_E2A_PORT, Motor_E2A_PIN))
                {
                    if (DL_GPIO_readPins(Motor_E2B_PORT, Motor_E2B_PIN))
                        Encoder2_Count--;
                    else
                        Encoder2_Count++;
                }
            }
        }
    }
}
