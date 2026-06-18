#include "ti_msp_dl_config.h"

#ifndef __PID__H
#define __PID__H


typedef struct{
	float Error0;
	float Error1;
	float ErrorInt;
	
	float Kp;
	float Ki;
	float Kd;
	
	float Target;
	float Actual;
	float Out;
	
	float OutMax;
	float OutMin;
	
	
}PID_t;

void PID_Update(PID_t *p);

#endif