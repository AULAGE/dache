#ifndef __STEPPER_H
#define __STEPPER_H

#include "system.h"

/* ================= 硬件 =================
 * PA6 = TIM3_CH1  PUL 脉冲
 * PA5 = GPIO      DIR 方向
 */
#define DIR_PIN          PAout(5)
#define DIR_UP_LEVEL     0        // 向上运动时 DIR 的电平；方向反了改成 0
#define DIR_SETUP_US     20       // 改变方向后等待多久再发脉冲

/* ================= 机械参数 ================= */
#define PULSES_PER_REV   1600     // 驱动器每圈脉冲数（电子齿轮比 / 细分）
#define LEAD_MM          70       // 电机每转一圈滑台移动的距离（mm）

// 每厘米脉冲数 ×1000（定点，避免浮点）：1600*10000/70 = 228571
#define STEPS_PER_CM_X1000  ((u32)PULSES_PER_REV * 10000UL / LEAD_MM)

/* ================= 行程 =================
 * 上电时滑台必须在最下端，此时位置 = 0
 */
#define MAX_POS_CM       50
#define MAX_POS_STEPS    ((s32)(((u32)MAX_POS_CM * STEPS_PER_CM_X1000 + 500) / 1000))

/* ================= 速度 / 加速度 =================
 * 速度单位：0.01cm/s（800 = 8.00cm/s）
 */
#define START_HZ            400   // 起跳频率（Hz），约 1.75cm/s
#define DEFAULT_SPEED_CM100 800   // 默认匀速 8.00cm/s
#define MIN_SPEED_CM100     50    // V 指令允许的最小速度 0.50cm/s
#define MAX_SPEED_CM100     2000  // V 指令允许的最大速度 20.00cm/s
#define ACCEL_CM_S2         50    // 加速度 / 减速度（cm/s^2）

#define ACCEL_SPS2       ((u32)ACCEL_CM_S2 * STEPS_PER_CM_X1000 / 1000)  // 脉冲/s^2

/* ================= API ================= */
void Stepper_Init(void);

// 设定目标位置（脉冲）。空闲时立即启动；运动中随时可改，
// 会平滑加速 / 减速，必要时先减速再掉头。调用方保证目标在 0~MAX_POS_STEPS 内
void Stepper_SetTarget(s32 target);
s32  Stepper_GetTarget(void);

void Stepper_Stop(void);       // 减速停止
void Stepper_Halt(void);       // 立即停止（最多再走完当前这一个脉冲，计数不丢）

u8   Stepper_IsBusy(void);
s32  Stepper_GetPosition(void);       // 单位：脉冲
void Stepper_SetPosition(s32 steps);  // 停机时调用，运动中调用无效

void Stepper_SetSpeedHz(u32 hz);      // 运动中也立即生效（平滑过渡）
u32  Stepper_GetSpeedHz(void);

#endif
