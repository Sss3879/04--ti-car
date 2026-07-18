#include "PID.h"

void PID_Update(PID_t *p)
{
	float integral;
	float output;

	p->Error1 = p->Error0;
	p->Error0 = p->Target - p->Actual;

	/* 条件积分：输出饱和时抑制积分继续累积。 */
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
