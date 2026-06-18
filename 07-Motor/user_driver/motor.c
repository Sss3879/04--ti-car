#include "motor.h"

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
}
void Motor_SetDuty(uint8_t motor, int32_t duty)
{

    if(motor==2) // 右电机
    {
        if(duty>=0)
        {   
            DL_GPIO_setPins(Motor_AIN1_PORT, Motor_AIN1_PIN) ;
            DL_GPIO_clearPins(Motor_AIN2_PORT,Motor_AIN2_PIN);
            DL_Timer_setCaptureCompareValue(PWM1_INST,duty,GPIO_PWM1_C0_IDX);
        }
        else
        {
            DL_GPIO_clearPins(Motor_AIN1_PORT, Motor_AIN1_PIN) ;
            DL_GPIO_setPins(Motor_AIN2_PORT,Motor_AIN2_PIN);
            DL_Timer_setCaptureCompareValue(PWM1_INST,-duty,GPIO_PWM1_C0_IDX);
        }

    }
    else if(motor==1) // 左电机
    {
        if(duty>=0)
        {
            DL_GPIO_setPins(Motor_BIN1_PORT, Motor_BIN1_PIN) ;
            DL_GPIO_clearPins(Motor_BIN2_PORT,Motor_BIN2_PIN);
            DL_Timer_setCaptureCompareValue(PWM1_INST,duty,GPIO_PWM1_C1_IDX);
        }
        else 
        {
            DL_GPIO_clearPins(Motor_BIN1_PORT, Motor_BIN1_PIN) ;
            DL_GPIO_setPins(Motor_BIN2_PORT,Motor_BIN2_PIN);
            DL_Timer_setCaptureCompareValue(PWM1_INST,-duty,GPIO_PWM1_C1_IDX);
        }
        

    }
}

