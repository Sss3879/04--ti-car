#include "buzzer.h"



/*
    定时器时钟:
    
    10MHz

    一个计数:
    
    1/10000000
    =
    0.1us

*/


void Buzzer_Init(void)
{

    /*
        初始化已经在

        SYSCFG_DL_init()

        中完成

    */


    //停止PWM
    DL_TimerG_stopCounter(buzzer_INST);


}




/*
    开启蜂鸣器
*/
void Buzzer_ON(void)
{

    DL_TimerG_startCounter(buzzer_INST);

}



/*
    关闭蜂鸣器
*/
void Buzzer_OFF(void)
{

    DL_TimerG_stopCounter(buzzer_INST);

}



/*
    设置频率

    f = TimerClock / (Period+1)

    TimerClock = 10MHz

*/

void Buzzer_SetFreq(uint32_t freq)
{

    uint32_t period;


    period = buzzer_INST_CLK_FREQ / freq;


    period = period - 1;



    /*
        设置PWM周期

    */

    DL_Timer_setLoadValue(
        buzzer_INST,
        period
    );



    /*
        默认50%

    */

    DL_Timer_setCaptureCompareValue(
    buzzer_INST,
    period / 2,
    GPIO_buzzer_C0_IDX
    );


}



/*
    设置占空比

    duty:
    0~100

*/

void Buzzer_SetDuty(uint8_t duty)
{

    uint32_t period;

    uint32_t compare;



    period = DL_Timer_getLoadValue(buzzer_INST);



    compare = period * duty / 100;



    DL_Timer_setCaptureCompareValue(
        buzzer_INST,
        compare,
        GPIO_buzzer_C0_IDX
    );

}