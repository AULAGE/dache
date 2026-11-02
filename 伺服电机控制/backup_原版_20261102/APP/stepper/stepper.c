#include "stepper.h"
#include "SysTick.h" 

// ============ 内部状态变量 ============
static volatile u32 s_total    = 0;   // 总步数
static volatile u32 s_current  = 0;   // 已走步数
static volatile u32 s_remain   = 0;   // 剩余步数
static volatile u32 s_acc      = 0;   // 加速段步数
static volatile u32 s_dec      = 0;   // 减速段步数
static volatile u32 s_dec_start = 0;  // 减速起始位置
static volatile u16 s_arr_start = 0;  // 起始ARR(慢)
static volatile u16 s_arr_run   = 0;  // 匀速ARR(快)
static volatile u32 s_acc_step = 0; 
static volatile u32 s_dec_step = 0;
static volatile s32 s_position = 0;  // 当前位置（步数，带符号）
static volatile u8  s_dir = 0;
static volatile u32 s_acc_accum = 0;   // 加速累加器
static volatile u32 s_dec_accum = 0;   // 减速累加器
static volatile u16 s_cur_arr   = 0;   // 当前ARR值

// ============ 初始化 ============
void Stepper_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef timbase;
    TIM_OCInitTypeDef       oc;
    NVIC_InitTypeDef        nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // PA6 = TIM3_CH1 复用推挽 (PUL脉冲输出)
    gpio.GPIO_Pin   = GPIO_Pin_6;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    // PA5 = DIR方向 推挽输出
    gpio.GPIO_Pin  = GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &gpio);

    // TIM3: 72MHz / 72 = 1MHz → 1个计数 = 1us
    timbase.TIM_Prescaler     = 72 - 1;
    timbase.TIM_CounterMode   = TIM_CounterMode_Up;
    timbase.TIM_Period        = START_PERIOD - 1;
    timbase.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &timbase);

    // PWM1模式，50%占空比
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState  = TIM_OutputState_Enable;
    oc.TIM_Pulse        = START_PERIOD / 2;
    oc.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &oc);

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    // 中断优先级
    nvic.NVIC_IRQChannel                   = TIM3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM3, DISABLE);  // 默认关闭，等命令再启动
}

// ============ 启动运动 ============
void Stepper_Move(u8 dir, u32 total_steps)
{
		s32 cur;
	  if(s_remain > 0) return;
		if(total_steps == 0) return;
		
		cur = s_position;
		if(total_steps == 0) return;
	
		GPIO_ResetBits(GPIOA, GPIO_Pin_6);
	
		s_dir = dir;
    DIR_PIN = (dir == 1) ? DIR_FORWARD : DIR_REVERSE;
    delay_us(20);
	
    if(dir == 1 && cur + (s32)total_steps > (s32)MAX_POS_STEPS)
        total_steps = (u32)((s32)MAX_POS_STEPS - cur);
    if(dir == 0 && cur - (s32)total_steps < (s32)MIN_POS_STEPS)
        total_steps = (u32)(cur - (s32)MIN_POS_STEPS);

    s_total   = total_steps;
    s_current = 0;
    s_remain  = total_steps;

    // ★ 先赋值 ARR 范围
    s_arr_start = START_PERIOD - 1;
    s_arr_run   = RUN_PERIOD - 1;

    // 再算加减速段
    s_acc = total_steps * ACC_PERCENT / 100;
    s_dec = total_steps * DEC_PERCENT / 100;
    if(s_acc + s_dec >= total_steps)
    {
        s_acc = total_steps / 2;
        s_dec = total_steps - s_acc;
    }
    if(s_acc == 0) s_acc = 1;
    if(s_dec == 0) s_dec = 1;
    s_dec_start = total_steps - s_dec;

    // ★ 最后算步进增量（此时所有值都已确定）
    s_acc_step = ((u32)(s_arr_start - s_arr_run) * 1000) / s_acc;
    s_dec_step = ((u32)(s_arr_start - s_arr_run) * 1000) / s_dec;

		// 1. 关闭预装载，使ARR立即生效（设定第一个周期）
		// 2. 写入初始周期
		// 3. 开启预装载，后续ISR中修改ARR在UEV时才更新（避免毛刺）
    TIM_ARRPreloadConfig(TIM3, DISABLE);
    TIM3->ARR  = s_arr_start;
    TIM3->CCR1 = (s_arr_start + 1) / 2;
    TIM3->CNT  = 0;
    TIM_ARRPreloadConfig(TIM3, ENABLE);
		
		s_acc_accum = 0;
		s_dec_accum = 0;
		s_cur_arr   = s_arr_start;
		
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
		
		
}

// ============ 查询是否在运动 ============
u8 Stepper_IsBusy(void)
{
    return (s_remain > 0) ? 1 : 0;
}

// ============ TIM3中断：每个脉冲进一次 ============
void TIM3_IRQHandler(void)
{
    u16 new_arr;

    if(TIM_GetITStatus(TIM3, TIM_IT_Update) == RESET) return;
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    if(s_remain == 0)
    {
        TIM_Cmd(TIM3, DISABLE);
        TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
        return;
    }

    s_remain--;
    s_current++;
		
		if(s_dir == 1)
        s_position++;
    else
        s_position--;
		
    // ISR 中替换整个加减速计算块
		if(s_current <= s_acc)
		{
    s_acc_accum += s_acc_step;
    while(s_acc_accum >= 1000)
    {
        s_acc_accum -= 1000;
        if(s_cur_arr > s_arr_run) s_cur_arr--;
    }
    new_arr = s_cur_arr;
		}
		else if(s_current > s_dec_start)
		{
    s_dec_accum += s_dec_step;
    while(s_dec_accum >= 1000)
    {
        s_dec_accum -= 1000;
        if(s_cur_arr < s_arr_start) s_cur_arr++;
    }
    new_arr = s_cur_arr;
		}
		else
		{
		s_cur_arr = s_arr_run;
    new_arr = s_arr_run;
		}

    TIM3->ARR  = new_arr;
    TIM3->CCR1 = (new_arr + 1) / 2;  // 保持50%占空比

    // 最后一步，停止定时器
    if(s_remain == 0)
    {
        TIM_Cmd(TIM3, DISABLE);
        TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			  GPIO_ResetBits(GPIOA, GPIO_Pin_6);  // 强制拉低PUL
				s_cur_arr = s_arr_start;
    }
}
void Stepper_Stop(void)
{
    __disable_irq();
    TIM_Cmd(TIM3, DISABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    s_remain  = 0;
    s_current = 0;
    __enable_irq();
    GPIO_ResetBits(GPIOA, GPIO_Pin_6);
}

s32 Stepper_GetPosition(void) { return s_position; }
void Stepper_ResetPosition(void) { s_position = 0; }
