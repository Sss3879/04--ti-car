#include "PID.h"

void PID_Update(PID_t *p)
{
    // 滑动误差
    p->Error2 = p->Error1;
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;
	

    // 增量式核心公式
    float dOut = p->Kp * (p->Error0 - p->Error1)
               + p->Ki * p->Error0
               + p->Kd * (p->Error0 - 2.0f * p->Error1 + p->Error2);

    p->Out += dOut;
	
	if(p->Out> p->OutMax){p->Out= p->OutMax;}
	if(p->Out< p->OutMin){p->Out= p->OutMin;}
}
void PID_Position(PID_t *p)
{
    p->Error0 = p->Target - p->Actual;

    p->Integral += p->Error0;

    p->Out =
        p->Kp * p->Error0 +
        p->Ki * p->Integral +
        p->Kd * (p->Error0 - p->Error1);

    p->Error1 = p->Error0;

    if(p->Out > p->OutMax) p->Out = p->OutMax;
    if(p->Out < p->OutMin) p->Out = p->OutMin;
}
