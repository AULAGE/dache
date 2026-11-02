#include "sg90.h"

#define SERVO_MIN   500
#define SERVO_MAX   2500
#define SERVO_MID   1500

void SG90_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    // 1. 使能所需时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);

    // 2. 禁用JTAG，释放PB3/PB4等引脚（保留SWD调试）
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    // 3. TIM2部分重映射1：将CH2映射到PB3
    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);

    // 4. TIM3部分重映射：将CH1/CH2映射到PB4/PB5
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);

    // 5. 配置四个GPIO引脚为复用推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;

    // PB1 (TIM3_CH4)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PB3 (TIM2_CH2)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PB4 (TIM3_CH1)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PB5 (TIM3_CH2)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 6. 初始化TIM2（用于PB3）
    TIM_TimeBaseStruct.TIM_Period = 19999;               // 20ms周期
    TIM_TimeBaseStruct.TIM_Prescaler = 71;               // 72MHz/72 = 1MHz计数
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);

    // 配置TIM2通道2 (PB3) PWM
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = SERVO_MID;              // 初始中值
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM2, &TIM_OCInitStruct);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    // 7. 初始化TIM3（用于PB1, PB4, PB5）
    TIM_TimeBaseStruct.TIM_Period = 19999;
    TIM_TimeBaseStruct.TIM_Prescaler = 71;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);

    // 配置TIM3通道1 (PB4)
    TIM_OCInitStruct.TIM_Pulse = SERVO_MID;
    TIM_OC1Init(TIM3, &TIM_OCInitStruct);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // 配置TIM3通道2 (PB5)
    TIM_OCInitStruct.TIM_Pulse = SERVO_MID;
    TIM_OC2Init(TIM3, &TIM_OCInitStruct);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // 配置TIM3通道4 (PB1)
    TIM_OCInitStruct.TIM_Pulse = SERVO_MID;
    TIM_OC4Init(TIM3, &TIM_OCInitStruct);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

// 舵机A: PB4 (TIM3_CH1)
void SG90A_SetAngle(float Angle)
{
    if (Angle < 0.0f) Angle = 0.0f;
    if (Angle > 180.0f) Angle = 180.0f;
    uint16_t pulse = (uint16_t)(500.0f + Angle / 180.0f * 2000.0f);
    TIM_SetCompare1(TIM3, pulse);
}

// 舵机B: PB3 (TIM2_CH2)
void SG90B_SetAngle(float Angle)
{
    if (Angle < 0.0f) Angle = 0.0f;
    if (Angle > 180.0f) Angle = 180.0f;
    uint16_t pulse = (uint16_t)(500.0f + Angle / 180.0f * 2000.0f);
    TIM_SetCompare2(TIM2, pulse);
}

// 舵机C: PB5 (TIM3_CH2)
void SG90C_SetAngle(float Angle)
{
    if (Angle < 0.0f) Angle = 0.0f;
    if (Angle > 180.0f) Angle = 180.0f;
    uint16_t pulse = (uint16_t)(500.0f + Angle / 180.0f * 2000.0f);
    TIM_SetCompare2(TIM3, pulse);
}

// 舵机D: PB1 (TIM3_CH4)
void SG90D_SetAngle(float Angle)
{
    if (Angle < 0.0f) Angle = 0.0f;
    if (Angle > 180.0f) Angle = 180.0f;
    uint16_t pulse = (uint16_t)(500.0f + Angle / 180.0f * 2000.0f);
    TIM_SetCompare4(TIM3, pulse);
}

