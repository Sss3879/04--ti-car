/* ========================================================================
 *                              16--package
 *                         MSPM0G3507 智能车总控
 * ========================================================================
 *  接线说明：
 *    OLED        SDA=PB3  SCL=PB2
 *    Serial      TX=PA28  RX=PA31  (115200)
 *    Blue_Serial TX=PA26  RX=PA25  (9600 蓝牙)
 *    Key          Key1=PA24  Key2=PA23
 *    POT         PA16 (ADC)
 *    ICM42688    SDA=PA0  SCL=PA1
 *    Buzzer      (syscfg配置)
 * ======================================================================== */

/* ========================== 头文件 ========================== */
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
#include "uart_vofa.h"
#include "icm42688.h"
#include "IMU.h"
#include "buzzer.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"

/* ========================== PID 参数 ========================== */
PID_t Motor_Left = {
    .Kp = 4.56f, .Ki = 1.01f, .Kd = 0,
    .Target = 0, .OutMax = 2500, .OutMin = -2500,
};

PID_t Motor_Right = {
    .Kp = 3.97f, .Ki = 0.95f, .Kd = 0,
    .Target = 0, .OutMax = 2500, .OutMin = -2500,
};

PID_t Angle_PID = {
    .Kp = 2.17, .Ki = 0, .Kd = 0.28,
    .Target = 0, .OutMax = 150, .OutMin = -150,
};

/* ========================== 运行模式 ========================== */
typedef enum {
    MODE_STOP = 0,
    MODE_ANGLE,
    MODE_LINE,
    MODE_RECT
} RunMode_t;

/******************** 矩形路线状态定义 ********************/
typedef enum {
    RECT_AB_LINE = 0,
    RECT_B_TURN,
    RECT_B_FIND_LINE,
    RECT_BC_LINE,
    RECT_C_TURN,
    RECT_C_FIND_LINE,
    RECT_CD_LINE,
    RECT_D_TURN,
    RECT_D_FIND_LINE,
    RECT_DA_LINE,
    RECT_STOP
} RoadState_t;

/* ========================== 全局变量 ========================== */
volatile uint8_t status = 0;
volatile int32_t Encoder1_Count = 0;
volatile int32_t Encoder2_Count = 0;

extern float speed_1;
extern float speed_2;
extern uint8_t printf_flag;
extern int32_t dbg_c1, dbg_c2;
extern volatile uint32_t motor_irq_count;
extern volatile uint32_t motor_pid_count;

char buffer[64];
float ypr[3];
uint8_t ICM42688_UpdateFlag = 0;
RunMode_t RunMode = MODE_STOP;

int16_t BaseSpeed = 200;

#define SPEED_MAX  250
#define SPEED_MIN -250

static uint8_t DisplayTick = 0;

/* ========================== 矩形路线参数 ========================== */
RoadState_t RoadState = RECT_AB_LINE;
float StartAngle = 0;
uint8_t Lost_Count = 0;
uint8_t FindLine_Count = 0;

#define TURN_ANGLE_DEG        40.0f
#define TURN_SPEED            50
#define FIND_SPEED            40
#define LOST_CONFIRM_COUNT    2
#define FIND_CONFIRM_COUNT    3

/* ========================== 灰度传感器 ========================== */
	/* 与 10-GraySensor 当前实测标定值保持一致。 */
	unsigned short white[8] = {1315,1300,1331,1347,1609,1604,1739,1746   };
	unsigned short black[8] = {350,350,350,350,350,350  };
unsigned short Anolog[8] = {0};
unsigned short Normal[8] = {0};
No_MCU_Sensor sensor;
unsigned char Digtal;
int16_t Line_Error = 0;
uint8_t Line_LostFlag = 0;
float Line_Kp = 0.03f;

/* ========================== 辅助函数 ========================== */
void delay_ms(uint32_t __ms);

static float Limit_Float(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}


void Car_LineControl(void)
{
    int16_t Turn;
    Line_Error   = GraySensor_GetLineError(&sensor);
    Line_LostFlag = GraySensor_GetLostFlag();

    Turn = (int16_t)(Line_Kp * Line_Error);
    if (Turn > 15)  Turn = 15;
    if (Turn < -15) Turn = -15;

    Motor_Left.Target  = Limit_Float(BaseSpeed + Turn, SPEED_MIN, SPEED_MAX);
    Motor_Right.Target = Limit_Float(BaseSpeed - Turn, SPEED_MIN, SPEED_MAX);
}

/* ========================== 矩形路线控制函数 ========================== */
static float MyAbsFloat(float x) { return (x >= 0) ? x : -x; }

void Car_TurnRightControl(void)
{
    Motor_Left.Target  = Limit_Float( TURN_SPEED, SPEED_MIN, SPEED_MAX);
    Motor_Right.Target = Limit_Float(-TURN_SPEED, SPEED_MIN, SPEED_MAX);
}

uint8_t GraySensor_AnyBlackDetected(void)
{
    No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
    if (!Get_Normalize_For_User(&sensor, Normal)) return 0;
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (Normal[i] < 1500) return 1;
    }
    return 0;
}

void Car_FindLineControl(void)
{
    int16_t Turn;
    if (GraySensor_AnyBlackDetected()) {
        Line_Error   = GraySensor_GetLineError(&sensor);
        Line_LostFlag = GraySensor_GetLostFlag();
        Turn = (int16_t)(Line_Kp * Line_Error);
        if (Turn > 6)  Turn = 6;
        if (Turn < -6) Turn = -6;
        Motor_Left.Target  = Limit_Float(FIND_SPEED + Turn, SPEED_MIN, SPEED_MAX);
        Motor_Right.Target = Limit_Float(FIND_SPEED - Turn, SPEED_MIN, SPEED_MAX);
    } else {
        Motor_Left.Target  = Limit_Float(FIND_SPEED, SPEED_MIN, SPEED_MAX);
        Motor_Right.Target = Limit_Float(FIND_SPEED, SPEED_MIN, SPEED_MAX);
    }
}

uint8_t Rect_CornerDetected(void)
{
    /* 用二值化结果判断丢线：8路全白(0xFF) = 丢线 */
    if (Digtal == 0xFF) {
        if (++Lost_Count >= LOST_CONFIRM_COUNT) { Lost_Count = 0; return 1; }
    } else {
        Lost_Count = 0;
    }
    return 0;
}

uint8_t Rect_FindLineDetected(void)
{
    /* 至少有一路看到黑(0) = 找到线 */
    if (Digtal != 0xFF) {
        if (++FindLine_Count >= FIND_CONFIRM_COUNT) { FindLine_Count = 0; return 1; }
    } else {
        FindLine_Count = 0;
    }
    return 0;
}

void Road_Task(void)
{
    switch (RoadState) {
        case RECT_AB_LINE:
            if (Rect_CornerDetected()) { StartAngle = ypr[0]; RoadState = RECT_B_TURN; }
            else { Car_LineControl(); }
            break;
        case RECT_B_TURN:
            Car_TurnRightControl();
            if (MyAbsFloat(ypr[0] - StartAngle) >= TURN_ANGLE_DEG) RoadState = RECT_B_FIND_LINE;
            break;
        case RECT_B_FIND_LINE:
            Car_FindLineControl();
            if (Rect_FindLineDetected()) { StartAngle = ypr[0]; RoadState = RECT_BC_LINE; }
            break;
        case RECT_BC_LINE:
            if (Rect_CornerDetected()) { StartAngle = ypr[0]; RoadState = RECT_C_TURN; }
            else { Car_LineControl(); }
            break;
        case RECT_C_TURN:
            Car_TurnRightControl();
            if (MyAbsFloat(ypr[0] - StartAngle) >= TURN_ANGLE_DEG) RoadState = RECT_C_FIND_LINE;
            break;
        case RECT_C_FIND_LINE:
            Car_FindLineControl();
            if (Rect_FindLineDetected()) { StartAngle = ypr[0]; RoadState = RECT_CD_LINE; }
            break;
        case RECT_CD_LINE:
            if (Rect_CornerDetected()) { StartAngle = ypr[0]; RoadState = RECT_D_TURN; }
            else { Car_LineControl(); }
            break;
        case RECT_D_TURN:
            Car_TurnRightControl();
            if (MyAbsFloat(ypr[0] - StartAngle) >= TURN_ANGLE_DEG) RoadState = RECT_D_FIND_LINE;
            break;
        case RECT_D_FIND_LINE:
            Car_FindLineControl();
            if (Rect_FindLineDetected()) { StartAngle = ypr[0]; RoadState = RECT_DA_LINE; }
            break;
        case RECT_DA_LINE:
            RoadState = RECT_AB_LINE;   // 跑完一圈，循环
            break;
        default:
            Motor_Left.Target  = 0;
            Motor_Right.Target = 0;
            break;
    }
}

/* ========================== OLED 状态显示 ========================== */
void OLED_DisplayStatus(void)
{
    char line[32];

    /* 第1行：模式 (y=0) */
    snprintf(line, sizeof(line), "Mode:%-5s",
             RunMode == MODE_STOP  ? "STOP" :
             RunMode == MODE_ANGLE ? "Angle" :
             RunMode == MODE_LINE  ? "Line" : "Rect");
    OLED_ShowString(0, 0, (u8 *)line, 12);

    /* 第2行：矩形子状态 (y=16) */
    snprintf(line, sizeof(line), "Road:%-2d %-8s",
             RoadState,
             RoadState == RECT_AB_LINE      ? "AB_Line" :
             RoadState == RECT_B_TURN       ? "B_Turn" :
             RoadState == RECT_B_FIND_LINE  ? "B_Find" :
             RoadState == RECT_BC_LINE      ? "BC_Line" :
             RoadState == RECT_C_TURN       ? "C_Turn" :
             RoadState == RECT_C_FIND_LINE  ? "C_Find" :
             RoadState == RECT_CD_LINE      ? "CD_Line" :
             RoadState == RECT_D_TURN       ? "D_Turn" :
             RoadState == RECT_D_FIND_LINE  ? "D_Find" :
             RoadState == RECT_DA_LINE      ? "DA_Line" : "STOP");
    OLED_ShowString(0, 16, (u8 *)line, 12);

    /* 第3行：实际速度 (y=32) */
    snprintf(line, sizeof(line), "La:%-5.1f Ra:%-5.1f  ",
             speed_1, speed_2);
    OLED_ShowString(0, 32, (u8 *)line, 12);

    /* 第4行：当前 yaw + 巡线误差 (y=48) */
    snprintf(line, sizeof(line), "Yaw:%-6.1f Err:%-4d  ",
             ypr[0], Line_Error);
    OLED_ShowString(0, 48, (u8 *)line, 12);

    OLED_Refresh();
}

/* ========================== 定时器初始化 ========================== */
void TimeA1_Init(void)
{
    NVIC_ClearPendingIRQ(TIMG6_INT_IRQn);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

/* ========================================================================
 *                              主函数
 * ======================================================================== */
int main(void)
{



    // // OLED_ColorTurn(0);
    // OLED_Clear();
    // uint32_t a=20;
    // char oled_str[40];
    // sprintf(oled_str, "a = %d", a);
    // OLED_ShowString(0, 0,(u8*) "hello world", 12);
    // OLED_ShowString(0, 13,(u8*) oled_str, 12);
    // OLED_Refresh();

    /* ---- 硬件初始化 ---- */
    SYSCFG_DL_init();
    delay_ms(100);                                      // 等待外设上电稳定
    Buzzer_Init();
    BlueSerial_Init();
    delay_ms(1000);                                       // 等待 OLED I2C 总线就绪
    OLED_Init();
    

    OLED_Clear();
    OLED_ShowString(0, 0,  (u8 *)"OLED Test OK", 12);
    OLED_ShowString(0, 16, (u8 *)"16--package", 12);
    OLED_ShowString(0, 32, (u8 *)"MSPM0G3507", 12);
    OLED_ShowString(0, 48, (u8 *)"Init...", 12);
    
    OLED_Refresh();
    delay_ms(1000);

    IMU_init();                                         // 陀螺仪零偏校准（保持车子不动！）
    No_MCU_Ganv_Sensor_Init(&sensor, white, black);     // 灰度传感器
    TimeA1_Init();

    /* ---- 启动 ADC + DMA（灰度传感器） ---- */
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&ADC0->ULLMEM.MEMRES[0]);
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&ADC_VALUE[0]);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_ADC12_startConversion(ADC12_0_INST);

    /* ---- 使能外设中断 ---- */
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(Motor_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(Serial_INST_INT_IRQN);

    /* ---- 等待传感器稳定 ---- */
    // Serial_Printf("Calibrating... keep still\r\n");
    BlueSerial_Printf("Calibrating...\r\n");
    delay_ms(2000);                                     // 给 IMU 2 秒稳定时间
    // Serial_Printf("---  Start ---\r\n");
    delay_ms(20);

    /* ---- 启动电机驱动 ---- */
    Motor_Init();

    /* ---- 主循环 ---- */
    while (1)
    {
        /* ---------- IMU 更新 + 控制输出 + 蓝牙输出 (20ms) ---------- */
        if (ICM42688_UpdateFlag == 1)
        {
            ICM42688_UpdateFlag = 0;

            /* 与 10-GraySensor 一致：控制计算前先刷新灰度数据。 */
            No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
            Digtal = Get_Digtal_For_User(&sensor);
            Get_Anolog_Value(&sensor, Anolog);
            Get_Normalize_For_User(&sensor, Normal);

            switch (RunMode)
            {
                case MODE_STOP:
                    Motor_Left.Target  = 0;
                    Motor_Right.Target = 0;
                    break;
                case MODE_ANGLE:
                    Angle_PID.Actual = ypr[0];
                    PID_Position(&Angle_PID);
                    Motor_Right.Target = BaseSpeed - Angle_PID.Out;
                    Motor_Left.Target  = BaseSpeed + Angle_PID.Out;
                    break;
                case MODE_LINE:
                    Car_LineControl();
                    break;
                case MODE_RECT:
                    Road_Task();
                    break;
            }

            // BlueSerial_Printf("[plot,%.2f,%.2f,%.2f,%.2f,%.2f]",
            //                   Angle_PID.Target,
            //                   Angle_PID.Actual,
            //                   Angle_PID.Out,
            //                   Motor_Left.Target,
            //                   Motor_Right.Target);

            /* OLED 每 60ms 刷新 + 灰度蓝牙打印 (3次 × 20ms) */
            if (++DisplayTick >= 3)
            {
                DisplayTick = 0;
                OLED_DisplayStatus();

                /* 灰度 + 电机调试（仅巡线时） */
                if (RunMode == MODE_RECT) {
                    BlueSerial_Printf("[Gray D:%d%d%d%d%d%d%d%d Err:%d L:%d "
                                      "Lt:%.0f Rt:%.0f "
                                      "La:%.1f Ra:%.1f Out:%.0f,%.0f "
                                      "S:%d LC:%d]\r\n",
                        (Digtal>>0)&1, (Digtal>>1)&1, (Digtal>>2)&1, (Digtal>>3)&1,
                        (Digtal>>4)&1, (Digtal>>5)&1, (Digtal>>6)&1, (Digtal>>7)&1,
                        Line_Error, Line_LostFlag,
                        Motor_Left.Target, Motor_Right.Target,
                        speed_1, speed_2,
                        Motor_Left.Out, Motor_Right.Out,
                        RoadState, Lost_Count);
                }
            }
        }

        /* -------------- */
        if (Serial_RxFlag == 1)
        {
            vofa_parse_packet(Serial_RxPacket);
            Serial_RxFlag = 0;
        }

        /* ---------- 蓝牙命令 (UART3) ---------- */
        if (BlueSerial_RxFlag == 1)
        {
            char *Tag = strtok(BlueSerial_RxPacket, ",");
            if (Tag)
            {
                /* --- key,Stop,down --- */
                if (strcmp(Tag, "key") == 0)
                {
                    char *Name   = strtok(NULL, ",");
                    char *Action = strtok(NULL, ",");
                    if (Name && Action && strcmp(Name, "Stop") == 0 && strcmp(Action, "down") == 0)
                    {
                        BaseSpeed = 0;
                        Motor_Left.Target  = 0;
                        Motor_Right.Target = 0;
                    }
                }
                /* --- slider,Name,Value --- */
                else if (strcmp(Tag, "slider") == 0)
                {
                    char *Name  = strtok(NULL, ",");
                    char *Value = strtok(NULL, ",");
                    if (Name && Value)
                    {
                             if (strcmp(Name, "AngleKp")   == 0) Angle_PID.Kp     = atof(Value);
                        else if (strcmp(Name, "AngleKi")   == 0) Angle_PID.Ki     = atof(Value);
                        else if (strcmp(Name, "AngleKd")   == 0) Angle_PID.Kd     = atof(Value);
                        else if (strcmp(Name, "BaseSpeed") == 0) BaseSpeed        = atof(Value);
                        else if (strcmp(Name, "LeftKp")    == 0) Motor_Left.Kp    = atof(Value);
                        else if (strcmp(Name, "LeftKi")    == 0) Motor_Left.Ki    = atof(Value);
                        else if (strcmp(Name, "LeftKd")    == 0) Motor_Left.Kd    = atof(Value);
                        else if (strcmp(Name, "RightKp")   == 0) Motor_Right.Kp   = atof(Value);
                        else if (strcmp(Name, "RightKi")   == 0) Motor_Right.Ki   = atof(Value);
                        else if (strcmp(Name, "RightKd")   == 0) Motor_Right.Kd   = atof(Value);
                    }
                }
                /* --- joystick,LH,LV,RH,RV --- */
                else if (strcmp(Tag, "joystick") == 0)
                {
                    char *LH = strtok(NULL, ",");
                    char *LV = strtok(NULL, ",");
                    char *RH = strtok(NULL, ",");
                    char *RV = strtok(NULL, ",");
                    if (LH && LV && RH && RV)
                    {
                        BaseSpeed         = atoi(LV) / 5.0f;
                        Angle_PID.Target  = atoi(RH) / 5.0f;
                    }
                }
            }
            BlueSerial_RxFlag = 0;
        }

        /* ---------- 按键处理 ---------- */
        if (status == 1)              // Key1: 切换模式
        {
            RunMode = (RunMode == MODE_STOP) ? MODE_RECT : MODE_STOP;   // STOP↔RECT
            if (RunMode == MODE_RECT) {
                RoadState = RECT_AB_LINE;
                StartAngle = ypr[0];
            }
            // Serial_Printf("Mode: %d\r\n", RunMode);
            BlueSerial_Printf("Mode: %d\r\n", RunMode);
            status = 0;
        }
        else if (status == 2)         // Key2: 紧急停止
        {
            RunMode = MODE_STOP;
            BaseSpeed = 0;
            Motor_Left.Target  = 0;
            Motor_Right.Target = 0;
            // Serial_Printf("STOP\r\n");
            BlueSerial_Printf("STOP\r\n");
            status = 0;
        }
    }
}

/* ========================================================================
 *                           中断服务函数
 * ======================================================================== */

/* ---- TIMER_0: 20ms 周期，读取 IMU ---- */
void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST))
    {
        case DL_TIMER_IIDX_ZERO:
            IMU_getYawPitchRoll(ypr);
            ICM42688_UpdateFlag = 1;
            break;
        default:
            break;
    }
}

/* ---- GPIO 中断：编码器 + 按键 ---- */
void GROUP1_IRQHandler(void)
{
    uint32_t gpioa = DL_GPIO_getPendingInterrupt(GPIOA);
    uint32_t gpiob = DL_GPIO_getPendingInterrupt(GPIOB);

    /* GPIOA: 编码器1 E1A=PA21 | Key1=PA24 | Key2=PA23 */
    switch (gpioa)
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
            status = 1;   // 切换 STOP↔RECT
            break;
        case Key_Key2_IIDX:
            status = 2;   // 紧急停止
            break;
        default:
            break;
    }

    /* GPIOB: 编码器2 E2A=PB19 */
    if (gpiob == Motor_E2A_IIDX)
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
