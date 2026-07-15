/*  新增了激光超时控制，多一个标志位计时器
    有速度限制，误差越大速度越快，误差小于死区就停转
*/


/*              OLED      接线   SDA  PB2 |SCl  PB3 */
/*              Serial USART0      接线   PA28|PA31*/


#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include "Serial.h"
#include "stepper_motor.h"
#include "filter.h"

/* ===== 滤波通道（X轴/电机1,  Y轴/电机2）===== */
Filter_Channel filter_x;
Filter_Channel filter_y;

/* 电机死区：滤波后误差绝对值小于此值就停转。需大于滤波器死区(3) */
#define MOTOR_DEAD_ZONE  5

/* 电机速度范围（度/秒），各轴独立 */
#define SPEED_MIN_X   5     /* X轴最低速度 */
#define SPEED_MAX_X   40    /* X轴最高速度 */
#define SPEED_MIN_Y   3     /* Y轴最低速度 */
#define SPEED_MAX_Y   40    /* Y轴最高速度 */

/* ===== 串口超时配置 ===== */
#define UART_TIMEOUT_MS     500     /* 超时时间：500ms 无数据 */
#define UART_TIMEOUT_LOOPS  (UART_TIMEOUT_MS / 20)  /* 折算为主循环次数 */

/* ===== 电机驱动辅助函数 ===== */

/**
 * @brief 根据误差值驱动单轴步进电机
 * @param error    滤波后的误差值（有符号，0=归中）
 * @param motor_id 电机编号 (1=X轴, 2=Y轴)
 */
static void Motor_DriveByError(int16_t error, uint8_t motor_id)
{
    /* 范围死区：滤波后残余 ±1~4 不会让电机一直转 */
    if (error < MOTOR_DEAD_ZONE && error > -MOTOR_DEAD_ZONE) {
        Stepper_Motor_Stop(motor_id);
        return;
    }

    /* 方向：error > 0 → 正向, error < 0 → 反向
     * 极性校正：电机1(X轴)反转，电机2(Y轴)保持 */
    uint8_t dir = (error > 0) ? 1 : 0;
    if (motor_id == 1) dir = !dir;
    Stepper_Motor_Dir_Set(dir, motor_id);

    /* 速度与误差绝对值成正比，各轴独立 */
    uint16_t abs_error = (error > 0) ? error : -error;
    uint8_t speed;
    uint8_t speed_min, speed_max;

    if (motor_id == 1) {
        speed_min = SPEED_MIN_X;
        speed_max = SPEED_MAX_X;
    } else {
        speed_min = SPEED_MIN_Y;
        speed_max = SPEED_MAX_Y;
    }

    /* 映射：误差 0~120 → 速度 speed_min ~ speed_max */
    if (abs_error > 120) abs_error = 120;
    speed = (uint8_t)(speed_min + (abs_error * (speed_max - speed_min)) / 120);
    if (speed < speed_min) speed = speed_min;

    Stepper_Motor_Set_AngleSpeed(speed, motor_id);
    Stepper_Motor_RunContinuous(motor_id);
}


int main(void)
{
    SYSCFG_DL_init();

    /* ----- 串口初始化 ----- */
    NVIC_EnableIRQ(Serial_INST_INT_IRQN);
    Serial_Printf("---  Start ---\r\n");

    /* ----- 步进电机云台初始化 ----- */
    Stepper_Motor_Init();

    /* ----- 滤波通道初始化 ----- */
    Filter_Init(&filter_x, FILTER_ALPHA_X, FILTER_DEAD_ZONE_X, FILTER_MAX_DELTA_X);
    Filter_Init(&filter_y, FILTER_ALPHA_Y, FILTER_DEAD_ZONE_Y, FILTER_MAX_DELTA_Y);

    /* ----- 超时检测变量 ----- */
    uint32_t last_rx_counter = 0;   /* 上次收到数据时的计数 */
    uint32_t timeout_ticks  = 0;    /* 连续无数据的循环次数 */
    uint8_t  timed_out      = 0;    /* 是否已触发超时（防重复） */

    while (1)
    {
        /* 收到 K230 发来的新一帧误差数据 */
        if (Serial_RxFlag == 1)
        {
            int16_t raw_x = Serial_GetXError();
            int16_t raw_y = Serial_GetYError();
            int16_t out_x, out_y;

            /* ----- 三步滤波流水线 ----- */
            out_x = Filter_Process(&filter_x, raw_x);
            out_y = Filter_Process(&filter_y, raw_y);

            /* ----- 调试输出 ----- */
            Serial_Printf("x:%d y:%d\r\n", out_x, out_y);

            /* ----- 驱动步进电机归中 ----- */
            Motor_DriveByError(out_x, 1);  /* 电机1 = X轴 */
            Motor_DriveByError(out_y, 2);  /* 电机2 = Y轴 */

            /* 更新超时跟踪：收到数据，重置计时 */
            last_rx_counter = Serial_GetRxCounter();
            timeout_ticks   = 0;
            timed_out       = 0;

            Serial_RxFlag = 0;
        }
        else
        {
            /* 无新数据时，检查是否超时 */
            if (Serial_GetRxCounter() > 0 && !timed_out)
            {
                timeout_ticks++;
                if (timeout_ticks >= UART_TIMEOUT_LOOPS)
                {
                    /* 超时：激光信号丢失，误差归零，电机停转 */
                    Serial_Printf("--- UART Timeout, stop ---\r\n");

                    Stepper_Motor_Stop(1);
                    Stepper_Motor_Stop(2);

                    Filter_Reset(&filter_x);
                    Filter_Reset(&filter_y);

                    timed_out = 1;  /* 防止重复触发 */
                }
            }
        }

        delay_ms(20);  /* 主循环周期 ≈ 20ms (50Hz) */
    }
}
