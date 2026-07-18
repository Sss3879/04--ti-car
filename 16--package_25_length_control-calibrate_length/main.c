/* ========================================================================
 *         16--package_25_length_control-calibrate_length — 编码器+角度标定
 *                              MSPM0G3507
 * ========================================================================
 *
 *  【用途】
 *    1. 推车测编码器脉冲数 → 标定距离
 *    2. 手动转车测陀螺仪角度 → 验证 90° 转弯
 *
 *  【操作步骤 — 测距离】
 *    1. 上电等 IMU 校准完成
 *    2. 按 Key1 清零编码器
 *    3. 推车 50cm
 *    4. 看 OLED 第3行 Avg 值 → 告诉 Claude
 *
 *  【操作步骤 — 测角度】
 *    1. 上电等 IMU 校准完成
 *    2. 按 Key2 清零 yaw 角
 *    3. 手动把车顺时针转 90°
 *    4. 看 OLED 第2行 Yaw 是否接近 90°
 *
 *  【注意】
 *    - 本工程不初始化电机，轮子和车身可自由转动
 *    - 左右轮编码器方向相反（一正一负），Avg 已取绝对值
 *
 *  【标定数据】
 *    50cm → 886 脉冲 → 100cm = 1772 脉冲
 *
 * ========================================================================
 *  Key1 = 清零编码器
 *  Key2 = 清零 yaw 角
 *  OLED: Yaw | E1/E2 | Avg/脉冲 | CM距离
 *  蓝牙: Yaw + E1 + E2 + Avg
 * ======================================================================== */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include "Serial.h"
#include "BlueSerial.h"
#include "PID.h"
#include "icm42688.h"
#include "IMU.h"

/* motor.o 依赖的空壳 */
PID_t Motor_Left  = {0};
PID_t Motor_Right = {0};

volatile uint8_t status = 0;
volatile int32_t Encoder1_Count = 0;
volatile int32_t Encoder2_Count = 0;
uint8_t Flag_20ms = 0;
float ypr[3];

void TimeA1_Init(void) {
    NVIC_ClearPendingIRQ(TIMG6_INT_IRQn);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

int main(void)
{
    SYSCFG_DL_init();
    delay_ms(100);
    BlueSerial_Init();
    delay_ms(1000);
    OLED_Init();

    IMU_init();     // 陀螺仪零偏校准（保持车子不动！）
    TimeA1_Init();

    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(Motor_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(Serial_INST_INT_IRQN);

    BlueSerial_Printf("Calib Start: enc+angle\r\n");

    {
        uint16_t oled_tick = 0, blue_tick = 0;
        while (1)
        {
            /* IMU 每 20ms 更新（高频率，保证 AHRS 积分正确） */
            if (Flag_20ms == 1) {
                Flag_20ms = 0;
                IMU_getYawPitchRoll(ypr);
                oled_tick++;
                blue_tick++;
            }

            /* Key1 = 清零编码器, Key2 = 无效 */
            if (status == 1) {
                Encoder1_Count = 0;
                Encoder2_Count = 0;
                BlueSerial_Printf("Encoder zeroed\r\n");
                status = 0;
            } else if (status == 2) {
                status = 0;
            }

            /* OLED 每 200ms 刷新 */
            if (oled_tick >= 10) {
                oled_tick = 0;
                char line[32];
                int32_t e1 = Encoder1_Count, e2 = Encoder2_Count;
                if (e1 < 0) e1 = -e1;
                if (e2 < 0) e2 = -e2;
                int32_t avg = (e1 + e2) / 2;
                uint32_t mm = (uint32_t)avg * 500u / 886u;

                snprintf(line, sizeof(line), "Yaw:%.1f", ypr[0]);
                OLED_ShowString(0, 0, (u8 *)line, 12);
                snprintf(line, sizeof(line), "E1:%d E2:%d",
                         (int)Encoder1_Count, (int)Encoder2_Count);
                OLED_ShowString(0, 16, (u8 *)line, 12);
                snprintf(line, sizeof(line), "Avg:%d Pul", (int)avg);
                OLED_ShowString(0, 32, (u8 *)line, 12);
                snprintf(line, sizeof(line), "Dist:%d.%d cm",
                         (int)(mm/10), (int)(mm%10));
                OLED_ShowString(0, 48, (u8 *)line, 12);
                OLED_Refresh();
            }

            /* 蓝牙每 500ms 打印 */
            if (blue_tick >= 25) {
                blue_tick = 0;
                int32_t e1 = Encoder1_Count, e2 = Encoder2_Count;
                if (e1 < 0) e1 = -e1;
                if (e2 < 0) e2 = -e2;
                int32_t avg = (e1 + e2) / 2;
                BlueSerial_Printf("Yaw:%.1f E1:%d E2:%d Avg:%d\r\n",
                                  ypr[0], (int)Encoder1_Count, (int)Encoder2_Count, (int)avg);
            }
        }
    }
}

/* ---- TIMER_0: 20ms 设标志位 ---- */
void TIMER_0_INST_IRQHandler(void) {
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO)
        Flag_20ms = 1;
}

/* ---- GPIO 中断：编码器 + 按键 ---- */
void GROUP1_IRQHandler(void)
{
    uint32_t iidx;
    while ((iidx = DL_GPIO_getPendingInterrupt(GPIOA)) != 0) {
        switch (iidx) {
            case Motor_E1A_IIDX:
                if (DL_GPIO_readPins(Motor_E1A_PORT, Motor_E1A_PIN)) {
                    delay_cycles(100);
                    if (DL_GPIO_readPins(Motor_E1A_PORT, Motor_E1A_PIN)) {
                        Encoder1_Count += DL_GPIO_readPins(Motor_E1B_PORT, Motor_E1B_PIN) ? -1 : 1;
                    }
                }
                break;
            case Key_Key1_IIDX: status = 1; break;
            case Key_Key2_IIDX: status = 2; break;
            default: break;
        }
    }
    while ((iidx = DL_GPIO_getPendingInterrupt(GPIOB)) != 0) {
        if (iidx == Motor_E2A_IIDX) {
            if (DL_GPIO_readPins(Motor_E2A_PORT, Motor_E2A_PIN)) {
                delay_cycles(100);
                if (DL_GPIO_readPins(Motor_E2A_PORT, Motor_E2A_PIN)) {
                    Encoder2_Count += DL_GPIO_readPins(Motor_E2B_PORT, Motor_E2B_PIN) ? -1 : 1;
                }
            }
        }
    }
}
