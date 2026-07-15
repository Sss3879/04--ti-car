#include "ti_msp_dl_config.h"
#ifndef __STEPPER_MOTOR__H
#define __STEPPER_MOTOR__H

void Stepper_Motor_Dir_Set(uint8_t direction, uint8_t motor_id);
void Stepper_Motor_Step_Set(uint8_t step, uint8_t motor_id);
void Stepper_Motor_Init(void);
void Stepper_Motor_Start(uint8_t stepper_id);
void Stepper_Motor_RunContinuous(uint8_t stepper_id);  /* 连续模式，直到Stop才停 */
void Stepper_Motor_Stop(uint8_t stepper_id);
void Stepper_Motor_Set_AngleSpeed(uint8_t speed, uint8_t stepper_id);
void Stepper_Motor_Set_Angle(uint8_t angle, uint8_t stepper_id);

#endif /* __STEPPER_MOTOR__H */