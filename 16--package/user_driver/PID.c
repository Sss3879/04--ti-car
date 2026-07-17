#include "PID.h"

void PID_Update(PID_t *p)
{
	float integral;
	float output;

	p->Error1 = p->Error0;
	p->Error0 = p->Target - p->Actual;

	/* 与 10-GraySensor 一致的条件积分位置式 PID。 */
	integral = (p->Ki != 0.0f) ? (p->ErrorInt + p->Error0) : 0.0f;
	output = p->Kp * p->Error0
	       + p->Ki * integral
	       + p->Kd * (p->Error0 - p->Error1);

	if ((output <= p->OutMax && output >= p->OutMin) ||
	    (output > p->OutMax && p->Error0 < 0.0f) ||
	    (output < p->OutMin && p->Error0 > 0.0f))
	{
		p->ErrorInt = integral;
	}

	output = p->Kp * p->Error0
	       + p->Ki * p->ErrorInt
	       + p->Kd * (p->Error0 - p->Error1);

	if (output > p->OutMax) output = p->OutMax;
	if (output < p->OutMin) output = p->OutMin;
	p->Out = output;
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
