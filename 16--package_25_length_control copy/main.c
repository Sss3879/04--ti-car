/* ========================================================================
 *                 16--package_length_control
 *               MSPM0G3507 编码器距离走 100×100 矩形
 * ========================================================================
 *  Key1 = STOP↔RECT
 *  Key2 = 紧急停止
 *  50cm = 886 脉冲   100cm = 1772 脉冲

    循环（每 20ms）：
    Flag_20ms → IMU读陀螺仪 → switch(RunMode)
                                ├─ STOP: Target=0
                                └─ RECT: Road_Task()
    状态机（8 个状态循环）：


    AB ──距离够?──→ B_TURN ──转角≥90°?──→ BC ──距离够?──→ C_TURN
    │                                        │
    └──────────── DA ←── D_TURN ←── CD ←────┘
                    ↑                       
                转角≥90°?    距离够?

 * ======================================================================== */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "Serial.h"
#include "BlueSerial.h"
#include "motor.h"
#include "PID.h"
#include "icm42688.h"
#include "IMU.h"
#include "buzzer.h"
int ImuMissCount ;


/* ========================== PID ========================== */
PID_t Motor_Left  = { .Kp=4.56f, .Ki=1.01f, .Kd=0, .Target=0, .OutMax=2500, .OutMin=-2500 };
PID_t Motor_Right = { .Kp=3.97f, .Ki=0.95f, .Kd=0, .Target=0, .OutMax=2500, .OutMin=-2500 };

/* ========================== 模式 ========================== */
typedef enum { MODE_STOP = 0, MODE_RECT } RunMode_t;
typedef enum {
    RECT_AB, RECT_B_TURN, RECT_BC, RECT_C_TURN,
    RECT_CD, RECT_D_TURN, RECT_DA, RECT_STOP
} RoadState_t;

/* ========================== 全局变量 ========================== */
volatile uint8_t status = 0;
volatile int32_t Encoder1_Count = 0;
volatile int32_t Encoder2_Count = 0;
volatile int32_t Enc1_Total = 0;      // 累计（不随PID清零）
volatile int32_t Enc2_Total = 0;      // 累计
extern float speed_1, speed_2;
extern float TTangles_gyro[7];   // 陀螺仪原始角速度 dps

float ypr[3];
float TurnAccum = 0;             // 陀螺z轴积分角度（稳定，不受AHRS影响）
uint8_t Flag_20ms = 0;
RunMode_t RunMode = MODE_STOP;
static uint8_t DisplayTick = 0;

/* ========================== 矩形参数 ========================== */
RoadState_t RoadState = RECT_AB;
float StartAngle = 0;
uint32_t StartEnc = 0;

#define STRAIGHT_PULSE  1791    // 100cm (实测 100cm=1791脉冲)
#define TURN_ANGLE_DEG   45.0f   // 试80°
#define TURN_SPEED       40
#define STRAIGHT_SPEED   100
#define SPEED_MAX       200
#define SPEED_MIN      -200

/* ========================== 辅助函数 ========================== */
void delay_ms(uint32_t __ms);
static float Limit_Float(float v, float min, float max) {
    if (v > max) return max; if (v < min) return min; return v;
}
static float MyAbs(float x) { return (x>=0)?x:-x; }

/* ---- 取编码器绝对值平均 ---- */
static uint32_t EncAvg(void) {
    int32_t e1 = Enc1_Total, e2 = Enc2_Total;
    if (e1 < 0) e1 = -e1; if (e2 < 0) e2 = -e2;
    return (uint32_t)((e1 + e2) / 2);
}

/* ---- 直行 ---- */
void Car_Straight(void) {
    Motor_Left.Target  = Limit_Float( STRAIGHT_SPEED, SPEED_MIN, SPEED_MAX);
    Motor_Right.Target = Limit_Float( STRAIGHT_SPEED, SPEED_MIN, SPEED_MAX);
}

/* ---- 原地右转 ---- */
void Car_TurnRight(void) {
    Motor_Left.Target  = Limit_Float( TURN_SPEED, SPEED_MIN, SPEED_MAX);
    Motor_Right.Target = Limit_Float(-TURN_SPEED, SPEED_MIN, SPEED_MAX);
}

/* ---- 距离检测 ---- */
uint8_t DistReached(void) { return EncAvg() >= StartEnc + STRAIGHT_PULSE; }

/* ---- 矩形状态机 ---- */
void Road_Task(void) {
    switch (RoadState) {
        case RECT_AB:
            if (DistReached()) { StartAngle = ypr[0]; RoadState = RECT_B_TURN; }
            else Car_Straight();
            break;
        case RECT_B_TURN:
            Car_TurnRight();
            if (MyAbs(ypr[0] - StartAngle) >= TURN_ANGLE_DEG) {
                StartEnc = EncAvg(); RoadState = RECT_BC;
            }
            break;
        case RECT_BC:
            if (DistReached()) { StartAngle = ypr[0]; RoadState = RECT_C_TURN; }
            else Car_Straight();
            break;
        case RECT_C_TURN:
            Car_TurnRight();
            if (MyAbs(ypr[0] - StartAngle) >= TURN_ANGLE_DEG) {
                StartEnc = EncAvg(); RoadState = RECT_CD;
            }
            break;
        case RECT_CD:
            if (DistReached()) { StartAngle = ypr[0]; RoadState = RECT_D_TURN; }
            else Car_Straight();
            break;
        case RECT_D_TURN:
            Car_TurnRight();
            if (MyAbs(ypr[0] - StartAngle) >= TURN_ANGLE_DEG) {
                StartEnc = EncAvg(); RoadState = RECT_DA;
            }
            break;
        case RECT_DA:
            if (DistReached()) { RoadState = RECT_STOP; }
            else Car_Straight();
            break;
        case RECT_STOP:
            Motor_Left.Target = Motor_Right.Target = 0;
            break;
    }
}

/* ---- OLED ---- */
void OLED_Display(void) {
    char l[32];
    snprintf(l, sizeof(l), "Mode:%-5s", RunMode==MODE_STOP?"STOP":"Rect");
    OLED_ShowString(0,0,(u8*)l,12);
    snprintf(l, sizeof(l), "S:%d %s", RoadState,
        RoadState==RECT_AB?"AB": RoadState==RECT_B_TURN?"B_TURN":
        RoadState==RECT_BC?"BC": RoadState==RECT_C_TURN?"C_TURN":
        RoadState==RECT_CD?"CD": RoadState==RECT_D_TURN?"D_TURN":
        RoadState==RECT_DA?"DA": RoadState==RECT_STOP?"STOP":"?");
    OLED_ShowString(0,16,(u8*)l,12);
    {
        uint32_t seg = EncAvg() - StartEnc;
        uint32_t mm  = seg * 1000u / 1791u;
        snprintf(l, sizeof(l), "Seg:%d.%d/100cm", (int)(mm/10), (int)(mm%10));
    }
    OLED_ShowString(0,32,(u8*)l,12);
    snprintf(l, sizeof(l), "Yaw:%.1f La:%.1f Ra:%.1f", ypr[0], speed_1, speed_2);
    OLED_ShowString(0,48,(u8*)l,12);
    OLED_Refresh();
}

/* ---- 定时器 ---- */
void TimeA1_Init(void) {
    NVIC_ClearPendingIRQ(TIMG6_INT_IRQn);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

/* ========================== main ========================== */
int main(void) {
    SYSCFG_DL_init(); delay_ms(100);
    Buzzer_Init(); BlueSerial_Init(); delay_ms(1000); OLED_Init();

    OLED_Clear();
    OLED_ShowString(0,0,(u8*)"Length Rect",12);
    OLED_ShowString(0,16,(u8*)"100x100cm",12);
    OLED_ShowString(0,32,(u8*)"Pulse=1329",12);
    OLED_ShowString(0,48,(u8*)"Init...",12);
    OLED_Refresh(); delay_ms(1000);

    IMU_init(); TimeA1_Init();
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(Motor_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(Serial_INST_INT_IRQN);
    BlueSerial_Printf("Calibrating...\r\n");
    delay_ms(2000);

    Motor_Init();

    while (1) {
        if (Flag_20ms == 1) {
            Flag_20ms = 0;
            IMU_getYawPitchRoll(ypr);  

            switch (RunMode) {
                case MODE_STOP: Motor_Left.Target = Motor_Right.Target = 0; break;
                case MODE_RECT: Road_Task(); break;
            }

            if (++DisplayTick >= 3) {
                DisplayTick = 0;
                OLED_Display();
                if (RunMode == MODE_RECT) {
                    {
                        uint32_t seg = EncAvg() - StartEnc;            // 本段脉冲
                        uint32_t mm  = seg * 1000u / 1791u;              // → 毫米×10
                        BlueSerial_Printf("[Rect S:%d Seg:%d.%d/100cm Yaw:%.1f "
                                          "Lt:%.0f Rt:%.0f]\r\n",
                            RoadState, (int)(mm/10), (int)(mm%10),
                            ypr[0], Motor_Left.Target, Motor_Right.Target);
                        BlueSerial_Printf("Miss=%lu\r\n",ImuMissCount);
                    }
                }
            }
        }

        if (status == 1) {
            RunMode = (RunMode==MODE_STOP) ? MODE_RECT : MODE_STOP;
            if (RunMode == MODE_RECT) {
                RoadState = RECT_AB;
                Enc1_Total = Enc2_Total = 0;
                StartEnc = 0;
            }
            BlueSerial_Printf("Mode:%d\r\n", RunMode);
            status = 0;
        } else if (status == 2) {
            RunMode = MODE_STOP;
            Motor_Left.Target = Motor_Right.Target = 0;
            BlueSerial_Printf("STOP\r\n");
            status = 0;
        }
    }
}

/* ========================== ISR ========================== */

void TIMER_0_INST_IRQHandler(void) {
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO) {
        // IMU_getYawPitchRoll(ypr);   // ISR内读IMU，保证严格50Hz
        if (Flag_20ms)
            ImuMissCount++;

        Flag_20ms = 1;
        
    }
}

void GROUP1_IRQHandler(void) {
    uint32_t iidx;
    while ((iidx = DL_GPIO_getPendingInterrupt(GPIOA)) != 0) {
        switch (iidx) {
            case Motor_E1A_IIDX:
                if (DL_GPIO_readPins(Motor_E1A_PORT, Motor_E1A_PIN)) {
                    delay_cycles(100);
                    if (DL_GPIO_readPins(Motor_E1A_PORT, Motor_E1A_PIN)) {
                        int32_t d = DL_GPIO_readPins(Motor_E1B_PORT, Motor_E1B_PIN) ? -1 : 1;
                        Encoder1_Count += d;
                        Enc1_Total     += d;
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
                    int32_t d = DL_GPIO_readPins(Motor_E2B_PORT, Motor_E2B_PIN) ? -1 : 1;
                    Encoder2_Count += d;
                    Enc2_Total     += d;
                }
            }
        }
    }
}
