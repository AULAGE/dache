#include "pwm.h"

// 静态函数：一次性初始化TIM4的时基和两个PWM通道
static void TIM4_PWM_Init(u16 per, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 1. 使能时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // 2. 配置PB6和PB7为复用推挽输出（关键！）
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. 初始化TIM4时基
    TIM_TimeBaseInitStructure.TIM_Period = per;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    // 4. 配置PWM输出模式（通道1和通道2共用模式参数）
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

    // 通道1 (PB6) 初始占空比0
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    // 通道2 (PB7) 初始占空比0
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

    // 5. 使能自动重装载和定时器
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

// 左轮PWM初始化（直接调用内部公共初始化）
void PWMA_Init(u16 per, u16 psc)
{
    // 只需调用一次，可放在Motor_Init中统一调用，此处留空或直接调用公共函数
    // 为了保持接口兼容，这里不做实际初始化，而是依赖Motor_Init中的TIM4_PWM_Init
}

// 右轮PWM初始化（同理）
void PWMB_Init(u16 per, u16 psc)
{
    // 同上
}

// 方向引脚初始化（PA0~3）
void Wheel_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(L_IN1_PORT_RCC, ENABLE);
    RCC_APB2PeriphClockCmd(L_IN2_PORT_RCC, ENABLE);
    RCC_APB2PeriphClockCmd(R_IN1_PORT_RCC, ENABLE);
    RCC_APB2PeriphClockCmd(R_IN2_PORT_RCC, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    // 左IN1 (PA1)
    GPIO_InitStructure.GPIO_Pin = L_IN1_PIN;
    GPIO_Init(L_IN1_PORT, &GPIO_InitStructure);
    GPIO_SetBits(L_IN1_PORT, L_IN1_PIN);

    // 左IN2 (PA0)
    GPIO_InitStructure.GPIO_Pin = L_IN2_PIN;
    GPIO_Init(L_IN2_PORT, &GPIO_InitStructure);
    GPIO_SetBits(L_IN2_PORT, L_IN2_PIN);

    // 右IN1 (PA2)
    GPIO_InitStructure.GPIO_Pin = R_IN1_PIN;
    GPIO_Init(R_IN1_PORT, &GPIO_InitStructure);
    GPIO_SetBits(R_IN1_PORT, R_IN1_PIN);

    // 右IN2 (PA3)
    GPIO_InitStructure.GPIO_Pin = R_IN2_PIN;
    GPIO_Init(R_IN2_PORT, &GPIO_InitStructure);
    GPIO_SetBits(R_IN2_PORT, R_IN2_PIN);
}

// 电机总初始化
void Motor_Init(u16 per, u16 psc)
{
    Wheel_Init();                // 初始化方向引脚
    TIM4_PWM_Init(per, psc);     // 一次性初始化TIM4的PWM（PB6,PB7）
    // 注意：PWMA_Init 和 PWMB_Init 不再需要单独调用
}

// 停止所有电机
void stop(void)
{
    // PWM占空比归零（TIM4）
    TIM_SetCompare1(TIM4, 0);
    TIM_SetCompare2(TIM4, 0);

    // 所有方向引脚置高（刹车/停止）
    L_IN1 = 1;
    L_IN2 = 1;
    R_IN1 = 1;
    R_IN2 = 1;
}

// 左轮速度控制
// speed > 0 正转，speed < 0 反转，speed = 0 停止
void Left_Speed(int speed)
{
    if (speed > 0)
    {
        L_IN1 = 1;
        L_IN2 = 0;
    }
    else if (speed < 0)
    {
        L_IN1 = 0;
        L_IN2 = 1;
    }
    else
    {
        L_IN1 = 0;
        L_IN2 = 0;
    }
    
    TIM_SetCompare1(TIM4, abs(speed));
}


void Right_Speed(int speed)
{
    if (speed > 0)
    {
        R_IN1 = 1;
        R_IN2 = 0;
    }
    else if (speed < 0)
    {
        R_IN1 = 0;
        R_IN2 = 1;
    }
    else
    {
        R_IN1 = 0;
        R_IN2 = 0;
    }
    // 设置占空比（使用TIM4通道2）
    TIM_SetCompare2(TIM4, abs(speed));
}
