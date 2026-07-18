#include "motor.h"
#include "PID.h"

extern PID_t Motor_Left;
extern PID_t Motor_Right;

void Motor_Init(void)
{
    DL_GPIO_setPins(Motor_STBY_PORT, Motor_STBY_PIN) ;
    DL_Timer_startCounter(PWM1_INST);   
// 右电机 AIN1 / AIN2 / PWM C0 这一组控制的是右电机
    DL_GPIO_setPins(Motor_AIN1_PORT, Motor_AIN1_PIN) ;
    DL_GPIO_setPins(Motor_AIN2_PORT,Motor_AIN2_PIN);
    DL_Timer_setCaptureCompareValue(PWM1_INST,0,GPIO_PWM1_C0_IDX);
// 左电机 BIN1 / BIN2 / PWM C1 这一组控制的是左电机
    DL_GPIO_setPins(Motor_BIN1_PORT, Motor_BIN1_PIN) ;
    DL_GPIO_setPins(Motor_BIN2_PORT,Motor_BIN2_PIN);
    DL_Timer_setCaptureCompareValue(PWM1_INST,0,GPIO_PWM1_C1_IDX);

    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
}
void Motor_SetDuty(uint8_t motor, int32_t duty)
{

    if(motor==2) // 右电机
    {
        if(duty>=0)
        {   
            DL_GPIO_clearPins(Motor_AIN1_PORT, Motor_AIN1_PIN) ;
            DL_GPIO_setPins(Motor_AIN2_PORT,Motor_AIN2_PIN);
            DL_Timer_setCaptureCompareValue(PWM1_INST,duty,GPIO_PWM1_C0_IDX);
        }
        else
        {
            DL_GPIO_setPins(Motor_AIN1_PORT, Motor_AIN1_PIN) ;
            DL_GPIO_clearPins(Motor_AIN2_PORT,Motor_AIN2_PIN);
            DL_Timer_setCaptureCompareValue(PWM1_INST,-duty,GPIO_PWM1_C0_IDX);
        }

    }
    else if(motor==1) // 左电机
    {
        if(duty>=0)
        {
            DL_GPIO_clearPins(Motor_BIN1_PORT, Motor_BIN1_PIN) ;
            DL_GPIO_setPins(Motor_BIN2_PORT,Motor_BIN2_PIN);
            DL_Timer_setCaptureCompareValue(PWM1_INST,duty,GPIO_PWM1_C1_IDX);
        }
        else 
        {
            DL_GPIO_setPins(Motor_BIN1_PORT, Motor_BIN1_PIN) ;
            DL_GPIO_clearPins(Motor_BIN2_PORT,Motor_BIN2_PIN);
            DL_Timer_setCaptureCompareValue(PWM1_INST,-duty,GPIO_PWM1_C1_IDX);
        }
        

    }
}
extern volatile int32_t Encoder1_Count;
extern volatile int32_t Encoder2_Count;
float speed_1=0;
float speed_2=0;
void calculate_speed(uint8_t motor_id)
{
    int32_t count;

    if (motor_id == 1) {
        count = Encoder1_Count;
        Encoder1_Count = 0;

        if (count > MAX_ENCODER_COUNT || count < -MAX_ENCODER_COUNT) {
            speed_1 = 0;
            return;
        }
        speed_1 = -(float)count / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 20;
    }

    else if (motor_id == 2) {
        count = Encoder2_Count;
        Encoder2_Count = 0;

        if (count > MAX_ENCODER_COUNT || count < -MAX_ENCODER_COUNT) {
            speed_2 = 0;
            return;
        }
        speed_2 = (float)count / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 20;
    }


}


void MOTOR_PID_INST_IRQHandler(void)
{
    //50s每次进入中断，计算一次速度
    //同时计算pid
    switch(DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
        case DL_TIMER_IIDX_LOAD:


            calculate_speed(1);
            calculate_speed(2);

            Motor_Left.Actual=speed_1;
			Motor_Right.Actual=speed_2;

            PID_Update(&Motor_Left);
            PID_Update(&Motor_Right);

            /* Encoder1 -> 左轮 -> PWM motor 2；Encoder2 -> 右轮 -> PWM motor 1 */
            Motor_SetDuty(2, (int32_t)Motor_Left.Out);
            Motor_SetDuty(1, (int32_t)Motor_Right.Out);
            break;
        default:
            break;
    }
}
