#ifndef __STEPPER_H
#define __STEPPER_H

#include "system.h"

#define DIR_PIN   PAout(5)

#define LEAD_SCREW_PITCH    70
#define MICROSTEP           8
#define STEPS_PER_CM_X1000  ((200UL * MICROSTEP * 10000) / LEAD_SCREW_PITCH)

#define START_PERIOD   800
#define RUN_PERIOD     500
#define ACC_PERCENT    25
#define DEC_PERCENT    25
#define DIR_FORWARD    1
#define DIR_REVERSE    0

// ★ 限位参数
#define MIN_POS_CM     0
#define MAX_POS_CM     48
#define MAX_POS_STEPS  ((u32)MAX_POS_CM * STEPS_PER_CM_X1000 / 1000)
#define MIN_POS_STEPS  ((u32)MIN_POS_CM * STEPS_PER_CM_X1000 / 1000)

void Stepper_Init(void);
void Stepper_Move(u8 dir, u32 total_steps);
void Stepper_Stop(void);
u8   Stepper_IsBusy(void);
s32  Stepper_GetPosition(void);
void Stepper_ResetPosition(void);

#endif
