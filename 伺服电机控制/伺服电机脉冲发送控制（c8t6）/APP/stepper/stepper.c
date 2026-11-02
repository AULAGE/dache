#include "stepper.h"
#include "SysTick.h"

/*
 * 脉冲发生原理
 * ------------
 * TIM3 计数频率 1MHz（1 个计数 = 1us），ARR 决定一个脉冲的周期。
 * 通道 1 用 PWM2 模式：每个周期前半段低、后半段高，周期结束（更新事件）时回到低电平。
 * 所以"一次更新中断 = 一个完整脉冲"，在更新事件处停下时 PUL 一定是低电平。
 * 空闲时通道强制输出低电平（Forced Inactive），上电和停机都不会残留高电平。
 *
 * 运动规划（目标跟踪，恒加速度）
 * ------------------------------
 * 状态量是速度的平方 fsq = f^2。每走一步：
 *   加速 fsq += 2a，减速 fsq -= 2a     （v^2 = v0^2 + 2as，即恒加速度）
 *   从当前速度减到 f0 需要 (fsq - f0^2) / 2a 步
 * 每步根据"离目标还有几步"决定加速、匀速还是减速；
 * 目标可以在运动中随时改，目标跑到身后时先减速到 f0，再在一个脉冲间隙里换方向。
 * 换方向时 DIR 改变到下一个上升沿至少有半个 f0 周期（400Hz 时为 1.25ms）。
 */

#define OC1M_FORCED_LOW  0x0040   // OC1M = 100：强制无效电平
#define OC1M_PWM2        0x0070   // OC1M = 111：PWM 模式 2

static volatile s32 s_pos      = 0;   // 当前位置（脉冲，向上为正）
static volatile s32 s_target   = 0;   // 目标位置
static volatile u8  s_up       = 0;   // 当前方向
static volatile u8  s_running  = 0;   // 定时器在发脉冲
static volatile u8  s_halt_req = 0;   // 走完当前脉冲后立即停
static volatile u32 s_fsq;            // 当前频率的平方
static volatile u32 s_frsq;           // 匀速频率的平方
static volatile u32 s_f0sq;           // 起跳 / 停止频率的平方
static volatile u32 s_frun_hz;        // 匀速频率
static const    u32 s_2a = 2 * ACCEL_SPS2;

// ---------- 内部工具 ----------
static u32 isqrt(u32 x)
{
    u32 r = 0, b = 1UL << 30;
    while(b > x) b >>= 2;
    while(b)
    {
        if(x >= r + b) { x -= r + b; r = (r >> 1) + b; }
        else           { r >>= 1; }
        b >>= 2;
    }
    return r;
}

static void set_period(u32 hz)
{
    u32 arr;
    if(hz < 16) hz = 16;
    arr = 1000000UL / hz - 1;
    if(arr > 0xFFFF) arr = 0xFFFF;
    TIM3->ARR  = (u16)arr;
    TIM3->CCR1 = (u16)((arr + 1) / 2);   // 50% 占空比
}

static void oc1_mode(u16 mode)
{
    TIM3->CCMR1 = (TIM3->CCMR1 & ~TIM_CCMR1_OC1M) | mode;
}

static void set_dir_pin(u8 up)
{
    DIR_PIN = up ? DIR_UP_LEVEL : !DIR_UP_LEVEL;
}

// 停定时器并强制 PUL 为低
static void halt_output(void)
{
    TIM3->CR1  &= ~TIM_CR1_CEN;
    TIM3->DIER &= ~TIM_DIER_UIE;
    oc1_mode(OC1M_FORCED_LOW);
    TIM3->SR = (u16)~TIM_SR_UIF;
}

static void finish(void)
{
    halt_output();
    s_running = 0;
    s_fsq     = s_f0sq;
}

static void accel(void)
{
    u32 f = s_fsq + s_2a;
    s_fsq = (f > s_frsq) ? s_frsq : f;
}

static void decel(void)
{
    s_fsq = (s_fsq > s_f0sq + s_2a) ? s_fsq - s_2a : s_f0sq;
}

/* ===== PLAN_BEGIN（仿真测试会提取这一段） =====
 * 每完成一个脉冲调用一次，决定下一个脉冲的速度和方向。
 * 返回 0 表示应当停下。
 */
static u8 plan_next(void)
{
    s32 dist  = s_target - s_pos;
    s32 ahead = s_up ? dist : -dist;                  // 沿当前方向还剩几步（<0 表示目标在身后）
    u32 ss    = (s_fsq > s_f0sq) ? (s_fsq - s_f0sq) / s_2a : 0;   // 减速到 f0 需要几步

    // 硬限位保护：不允许越过 0 和 MAX（正常不会触发）
    if((s_up && s_pos >= MAX_POS_STEPS) || (!s_up && s_pos <= 0))
    {
        if(ahead > 0) { s_target = s_pos; ahead = 0; }
        s_fsq = s_f0sq;
        ss    = 0;
    }

    if(ahead == 0 && ss <= 1) return 0;               // 到位

    if(ahead <= 0)
    {
        if(s_fsq <= s_f0sq)
        {
            s_up = !s_up;                             // 已减到 f0：掉头
            set_dir_pin(s_up);
        }
        else decel();                                 // 先减速（会稍微冲过目标再回来）
    }
    else if(ss >= (u32)ahead) decel();                // 该减速了
    else if(s_fsq > s_frsq)   decel();                // 速度被调低了
    else if(ss + 1 < (u32)ahead) accel();             // 还来得及：加速（到匀速后 accel 不再增加）
    return 1;
}
/* ===== PLAN_END ===== */

// ---------- 初始化 ----------
void Stepper_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef       oc;
    NVIC_InitTypeDef        nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    Stepper_SetSpeedHz((u32)DEFAULT_SPEED_CM100 * STEPS_PER_CM_X1000 / 100000UL);
    s_fsq = s_f0sq;

    // DIR：推挽输出
    gpio.GPIO_Pin   = GPIO_Pin_5;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    set_dir_pin(1);

    // TIM3：72MHz / 72 = 1MHz
    tb.TIM_Prescaler     = 72 - 1;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    tb.TIM_Period        = 1000000UL / START_HZ - 1;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &tb);

    oc.TIM_OCMode      = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = (1000000UL / START_HZ) / 2;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &oc);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    halt_output();   // 先把通道强制为低，再把引脚切到复用功能，避免上电毛刺

    // PUL：复用推挽（TIM3_CH1）
    gpio.GPIO_Pin  = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    nvic.NVIC_IRQChannel                   = TIM3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

// ---------- 从静止启动（仅在 s_running=0 时调用） ----------
static void start_motion(void)
{
    s_up = (s_target > s_pos) ? 1 : 0;
    set_dir_pin(s_up);
    delay_us(DIR_SETUP_US);

    s_fsq      = s_f0sq;
    s_halt_req = 0;
    set_period(isqrt(s_f0sq));
    TIM3->CNT = 0;
    TIM3->EGR = TIM_EGR_UG;          // 把 ARR/CCR1 装进影子寄存器
    TIM3->SR  = (u16)~TIM_SR_UIF;    // UG 会置 UIF，清掉
    oc1_mode(OC1M_PWM2);             // CNT=0 < CCR1，输出仍为低
    s_running = 1;
    TIM3->DIER |= TIM_DIER_UIE;
    TIM3->CR1  |= TIM_CR1_CEN;
}

void Stepper_SetTarget(s32 target)
{
    u8 running;

    if(target < 0) target = 0;
    if(target > MAX_POS_STEPS) target = MAX_POS_STEPS;

    // 先写目标再读运行状态：若此时在运行，之后的中断一定能看到新目标
    __disable_irq();
    s_target = target;
    running  = s_running;
    __enable_irq();

    if(!running && target != s_pos) start_motion();
}

// ---------- TIM3 更新中断：每完成一个脉冲进一次 ----------
void TIM3_IRQHandler(void)
{
    if(!(TIM3->SR & TIM_SR_UIF)) return;
    TIM3->SR = (u16)~TIM_SR_UIF;

    if(!s_running) { halt_output(); return; }

    if(s_up) s_pos++;
    else     s_pos--;

    if(s_halt_req)
    {
        s_halt_req = 0;
        s_target   = s_pos;
        finish();
        return;
    }

    if(!plan_next())
    {
        finish();            // 新周期刚开始，PUL 为低，立即停
        return;
    }
    set_period(isqrt(s_fsq));   // 预装载：下一个更新事件后生效
}

// ---------- 减速停止：把目标改成"刚好能停下的位置" ----------
void Stepper_Stop(void)
{
    u32 ss;
    s32 stop_at, ahead;

    __disable_irq();
    if(s_running)
    {
        ss      = (s_fsq > s_f0sq) ? (s_fsq - s_f0sq) / s_2a : 0;
        stop_at = s_pos + (s_up ? (s32)(ss + 1) : -(s32)(ss + 1));
        ahead   = s_up ? (s_target - s_pos) : (s_pos - s_target);
        // 原目标更近（已经在减速了）就不改
        if(!(ahead > 0 && ahead <= (s32)(ss + 1)))
        {
            if(stop_at < 0) stop_at = 0;
            if(stop_at > MAX_POS_STEPS) stop_at = MAX_POS_STEPS;
            s_target = stop_at;
        }
    }
    __enable_irq();
}

// ---------- 立即停止 ----------
void Stepper_Halt(void)
{
    __disable_irq();
    if(s_running)
    {
        if(GPIOA->IDR & GPIO_Pin_6)
        {
            // 正处在脉冲高电平：驱动器可能已经计了这一步，让它走完再停（最多半个周期）
            s_halt_req = 1;
        }
        else
        {
            TIM3->CR1 &= ~TIM_CR1_CEN;
            if(TIM3->SR & TIM_SR_UIF)   // 刚好有一个脉冲走完、中断还没来得及处理
                s_pos += s_up ? 1 : -1;
            finish();
            s_target = s_pos;
        }
    }
    __enable_irq();
}

void Stepper_SetSpeedHz(u32 hz)
{
    u32 f0 = (START_HZ < hz) ? START_HZ : hz;
    if(hz < 20) hz = 20;
    if(f0 < 20) f0 = 20;
    __disable_irq();
    s_frun_hz = hz;
    s_frsq    = hz * hz;
    s_f0sq    = f0 * f0;
    if(!s_running) s_fsq = s_f0sq;
    __enable_irq();
}

u8   Stepper_IsBusy(void)      { return s_running; }
s32  Stepper_GetPosition(void) { return s_pos; }
s32  Stepper_GetTarget(void)   { return s_target; }
u32  Stepper_GetSpeedHz(void)  { return s_frun_hz; }

void Stepper_SetPosition(s32 steps)
{
    __disable_irq();
    if(!s_running) { s_pos = steps; s_target = steps; }
    __enable_irq();
}
