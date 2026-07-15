#include "stepper_motor.h"

/* 连续模式标志：为1时跳过step_remain检查，电机持续运转直到主动Stop */
volatile uint8_t motor1_continuous = 0;
volatile uint8_t motor2_continuous = 0;

void Stepper_Motor_Init()
{
    DL_GPIO_setPins(stepper_motor_RST2_PORT, stepper_motor_RST2_PIN);
    DL_GPIO_setPins(stepper_motor_SLP2_PORT, stepper_motor_SLP2_PIN);
    DL_GPIO_setPins(stepper_motor_DCY2_PORT, stepper_motor_DCY2_PIN);
    DL_GPIO_setPins(stepper_motor_DIR2_PORT, stepper_motor_DIR2_PIN);

    DL_GPIO_setPins(stepper_motor_RST1_PORT, stepper_motor_RST1_PIN);
    DL_GPIO_setPins(stepper_motor_SLP1_PORT, stepper_motor_SLP1_PIN);
    DL_GPIO_setPins(stepper_motor_DCY1_PORT, stepper_motor_DCY1_PIN);
    DL_GPIO_setPins(stepper_motor_DIR1_PORT, stepper_motor_DIR1_PIN);
}

void Stepper_Motor_Dir_Set(uint8_t direction,uint8_t motor_id)
{
    //direction传入的是1为高电平，0为低电平
    if(motor_id==1)
    {
        if(direction==1)
        {
            DL_GPIO_setPins(stepper_motor_DIR1_PORT,stepper_motor_DIR1_PIN);
        }
        else
        {
            DL_GPIO_clearPins(stepper_motor_DIR1_PORT, stepper_motor_DIR1_PIN);
        }
    }
    else if(motor_id==2)
    {
        if(direction==1)
        {
            DL_GPIO_setPins(stepper_motor_DIR2_PORT, stepper_motor_DIR2_PIN);
        }
        else
        {
            DL_GPIO_clearPins(stepper_motor_DIR2_PORT, stepper_motor_DIR2_PIN);
        }
    }
} 
void Stepper_Motor_Start(uint8_t stepper_id)
{
    if(stepper_id==1)
    {
        DL_Timer_startCounter(DCC_PWM1_INST);
        NVIC_EnableIRQ(DCC_PWM1_INST_INT_IRQN);
    }
    else if(stepper_id==2)
    {
        DL_Timer_startCounter(DCC_PWM2_INST);
        NVIC_EnableIRQ(DCC_PWM2_INST_INT_IRQN);
    }

}

/**
 * @brief 连续模式启动 — 电机持续运转，不受 step_remain 限制
 *        直到调用 Stepper_Motor_Stop() 才停止
 */
void Stepper_Motor_RunContinuous(uint8_t stepper_id)
{
    if (stepper_id == 1) {
        motor1_continuous = 1;
    } else if (stepper_id == 2) {
        motor2_continuous = 1;
    }
    Stepper_Motor_Start(stepper_id);
}

void Stepper_Motor_Stop(uint8_t stepper_id)
{
    if(stepper_id==1)
    {
        motor1_continuous = 0;
        DL_Timer_stopCounter(DCC_PWM1_INST);
    }
    else if(stepper_id==2)
    {
        motor2_continuous = 0;
        DL_Timer_stopCounter(DCC_PWM2_INST);
    }
}


// 角速度设置 角度/s
void Stepper_Motor_Set_AngleSpeed(uint8_t speed, uint8_t stepper_id)
{
    if (stepper_id == 1) {
        // 根据速度设置PWM频率
        uint32_t frequency = (uint32_t)(speed / 0.05625); // 计算所需的PWM频率
        frequency = frequency > 0 ? frequency : 1;
        // float period_sec = 1.0f / frequency;
        // // 1 / DCC_100_PWM2_INST_CLK_FREQ 
        // // period_sec / (1 / DCC_100_PWM2_INST_CLK_FREQ);
        
        // 计算定时器溢出值
        uint32_t period = DCC_PWM1_INST_CLK_FREQ / frequency;
        period = period < 65536 ? period : 65535;
        period = period > 800 ? period : 800; 
        DL_Timer_setLoadValue(DCC_PWM1_INST, period);
        DL_Timer_setCaptureCompareValue(DCC_PWM1_INST, period / 2, GPIO_DCC_PWM1_C0_IDX); // 设置占空比为50%
    }
    if (stepper_id == 2) {
        // 根据速度设置PWM频率
        uint32_t frequency = (uint32_t)(speed / 0.05625); // 计算所需的PWM频率
        frequency = frequency > 0 ? frequency : 1;
        // float period_sec = 1.0f / frequency;
        // // 1 / DCC_100_PWM2_INST_CLK_FREQ 
        // // period_sec / (1 / DCC_100_PWM2_INST_CLK_FREQ);
        
        // 计算定时器溢出值
        uint32_t period = DCC_PWM2_INST_CLK_FREQ / frequency;
        period = period < 65536 ? period : 65535;
        period = period > 800 ? period : 800; 
        DL_Timer_setLoadValue(DCC_PWM2_INST, period);
        DL_Timer_setCaptureCompareValue(DCC_PWM2_INST, period / 2, GPIO_DCC_PWM2_C0_IDX); // 设置占空比为50%
    }
}
uint32_t step_remain_2 = 0;
uint32_t step_remain_1 = 0;

void Stepper_Motor_Set_Angle(uint8_t angle,uint8_t stepper_id)
{
    if(stepper_id==1)
    {
         step_remain_1 = (uint16_t)(angle / 0.05625); // 计算所需的步数
    }  
    if (stepper_id == 2) {
        // 计算所需的步数
         step_remain_2 = (uint32_t)(angle / 0.05625); // 计算所需的步数
    }
    Stepper_Motor_Start(stepper_id);
}



void DCC_PWM2_INST_IRQHandler()
{
    switch (DL_Timer_getPendingInterrupt(DCC_PWM2_INST))
    {
        case DL_TIMER_IIDX_LOAD:
            if (motor2_continuous) {
                /* 连续模式：不检查步数，一直发脉冲 */
            } else if (step_remain_2 == 0) {
                Stepper_Motor_Stop(2);
            } else {
                step_remain_2--;
            }
            break;
        default:
            break;
    }
}

void DCC_PWM1_INST_IRQHandler()
{
    switch (DL_Timer_getPendingInterrupt(DCC_PWM1_INST))
    {
        case DL_TIMER_IIDX_LOAD:
            if (motor1_continuous) {
                /* 连续模式：不检查步数，一直发脉冲 */
            } else if (step_remain_1 == 0) {
                Stepper_Motor_Stop(1);
            } else {
                step_remain_1--;
            }
            break;
        default:
            break;
    }
}