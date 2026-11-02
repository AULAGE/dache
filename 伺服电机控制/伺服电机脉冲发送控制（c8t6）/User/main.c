/*
 * 升降轴下位机（STM32F103C8T6）
 * 串口 USART1 115200 8N1，PA9=TX  PA10=RX
 * 协议见 readme.md。每行一条指令，\n 或 \r 或 \r\n 结尾，不区分大小写。
 */
#include "system.h"
#include "SysTick.h"
#include "usart.h"
#include "stepper.h"
#include "iwdg.h"
#include <stdio.h>

// 1：任何复位后都认为滑台在最下端（不安全，仅调试用）
// 0：只有断电重新上电才认为在最下端；按复位键、看门狗、下载程序后要先发 Z
#define HOMED_ON_ANY_RESET  0

#define LINE_MAX        48
#define MAX_POS_CM100   ((s32)MAX_POS_CM * 100)

static u8  s_homed   = 0;   // 位置是否可信
static u8  s_moving  = 0;   // 主循环记录：已发出 OK GO、还没发 DONE
static u8  s_stopped = 0;   // 本次运动被 S/E 打断
static s32 s_preset  = 0;   // M 设定的目标（脉冲）

/* ================= 单位换算（0.01cm <-> 脉冲，四舍五入） ================= */
static s32 cm100_to_steps(s32 c)
{
    u32 a = (u32)((c < 0) ? -c : c);
    s32 s = (s32)((a * STEPS_PER_CM_X1000 + 50000UL) / 100000UL);
    return (c < 0) ? -s : s;
}

static s32 steps_to_cm100(s32 s)
{
    u32 a = (u32)((s < 0) ? -s : s);
    s32 c = (s32)((a * 100000UL + STEPS_PER_CM_X1000 / 2) / STEPS_PER_CM_X1000);
    return (s < 0) ? -c : c;
}

static void print_cm(s32 c)
{
    if(c < 0) { putchar('-'); c = -c; }
    printf("%d.%02d", (int)(c / 100), (int)(c % 100));
}

/* ================= 解析 =================
 * 解析一个厘米数，允许正负号和最多 2 位小数："30" "12.5" "-3.25"
 * 成功返回 1，结果单位 0.01cm
 */
static u8 parse_cm100(const char **pp, s32 *out)
{
    const char *p = *pp;
    s32 ip = 0, fp = 0;
    u8  neg = 0, nd = 0, nf = 0;

    while(*p == ' ') p++;
    if(*p == '+' || *p == '-') { neg = (*p == '-'); p++; }
    while(*p >= '0' && *p <= '9')
    {
        if(++nd > 5) return 0;           // 整数部分最多 5 位，防溢出
        ip = ip * 10 + (*p++ - '0');
    }
    if(*p == '.')
    {
        p++;
        while(*p >= '0' && *p <= '9')
        {
            if(nf < 2) { fp = fp * 10 + (*p - '0'); nf++; }
            p++;
        }
    }
    if(nd == 0 && nf == 0) return 0;
    if(nf == 1) fp *= 10;
    if(*p != '\0' && *p != ' ') return 0;
    *out = (ip * 100 + fp) * (neg ? -1 : 1);
    *pp = p;
    return 1;
}

static u8 has_arg(const char *p)
{
    while(*p == ' ') p++;
    return *p != '\0';
}

/* ================= 运动 ================= */
static void report_done(void)
{
    printf("DONE ");
    print_cm(steps_to_cm100(Stepper_GetPosition()));
    printf(s_stopped ? " STOP\r\n" : "\r\n");
    s_moving  = 0;
    s_stopped = 0;
}

// 走到绝对位置（脉冲）。空闲时启动；运动中直接改目标，平滑过渡
static void go_to(s32 target, u8 clamped)
{
    printf("OK GO ");
    print_cm(steps_to_cm100(target));
    printf(clamped ? " CLAMP\r\n" : "\r\n");

    s_moving  = 1;
    s_stopped = 0;
    if(!Stepper_IsBusy() && target == Stepper_GetPosition())
    {
        report_done();                 // 已在目标：立即 DONE
        return;
    }
    Stepper_SetTarget(target);
}

static u8 check_can_move(void)
{
    if(!s_homed) { printf("ERR UNHOMED\r\n"); return 0; }
    return 1;
}

// 相对运动的基准：运动中以当前目标为准（连续发 R 会累加），空闲时以当前位置为准
static s32 move_base(void)
{
    return Stepper_IsBusy() ? Stepper_GetTarget() : Stepper_GetPosition();
}

static void err_range(void)
{
    printf("ERR RANGE 0.00-%d.00\r\n", MAX_POS_CM);
}

// 相对运动，超出行程就截断到边界
static void move_relative(s32 delta_cm100)
{
    s32 cur = move_base();
    s32 tgt;
    s32 c;
    u8  clamped = 0;

    if(delta_cm100 > MAX_POS_CM100 || delta_cm100 < -MAX_POS_CM100) { err_range(); return; }
    // 在 0.01cm 单位下相加再换算，连续 R 不会累积舍入误差
    c = steps_to_cm100(cur) + delta_cm100;
    if(c > MAX_POS_CM100) { c = MAX_POS_CM100; clamped = 1; }
    if(c < 0)             { c = 0;             clamped = 1; }
    tgt = cm100_to_steps(c);
    if(clamped && tgt == cur) { printf("ERR LIMIT\r\n"); return; }
    go_to(tgt, clamped);
}

/* ================= 指令处理 ================= */
static void print_help(void)
{
    printf("CMD: A <cm> abs | R <+-cm> rel | 1 <cm> up | 0 <cm> down\r\n");
    printf("     M <cm> set | G go | S stop | E halt | P pos\r\n");
    printf("     Z [cm] set pos | V [cm/s] speed | ? help\r\n");
}

static void handle_line(char *line)
{
    const char *p = line;
    char c;
    s32  v;

    while(*p == ' ') p++;
    if(*p == '\0') return;
    c = *p;
    if(c >= 'a' && c <= 'z') c -= 32;

    // 兼容旧格式：1 10 = 向上 10cm，0 10 = 向下 10cm
    if((c == '0' || c == '1') && p[1] == ' ')
    {
        p += 2;
        if(!parse_cm100(&p, &v) || v <= 0) { printf("ERR FORMAT\r\n"); return; }
        if(!check_can_move()) return;
        move_relative(c == '1' ? v : -v);
        return;
    }

    p++;   // 跳过指令字母
    switch(c)
    {
    case 'S':   // 减速停
        if(Stepper_IsBusy()) { Stepper_Stop(); s_stopped = 1; }
        printf("OK S\r\n");
        break;

    case 'E':   // 立即停
        if(Stepper_IsBusy()) { Stepper_Halt(); s_stopped = 1; }
        printf("OK E\r\n");
        break;

    case 'P':
        printf("POS ");
        print_cm(steps_to_cm100(Stepper_GetPosition()));
        if(!s_homed)              printf(" UNHOMED\r\n");
        else if(Stepper_IsBusy())
        {
            printf(" MOVING TGT ");
            print_cm(steps_to_cm100(Stepper_GetTarget()));
            printf("\r\n");
        }
        else                      printf(" IDLE\r\n");
        break;

    case 'A':   // 绝对定位
        if(!parse_cm100(&p, &v)) { printf("ERR FORMAT\r\n"); break; }
        if(v < 0 || v > MAX_POS_CM100) { err_range(); break; }
        if(!check_can_move()) break;
        go_to(cm100_to_steps(v), 0);
        break;

    case 'R':   // 相对运动
        if(!parse_cm100(&p, &v)) { printf("ERR FORMAT\r\n"); break; }
        if(!check_can_move()) break;
        move_relative(v);
        break;

    case 'M':   // 只设目标
        if(!parse_cm100(&p, &v)) { printf("ERR FORMAT\r\n"); break; }
        if(v < 0 || v > MAX_POS_CM100) { err_range(); break; }
        s_preset = cm100_to_steps(v);
        printf("OK M ");
        print_cm(v);
        printf("\r\n");
        break;

    case 'G':   // 走到 M 设定的目标
        if(!check_can_move()) break;
        go_to(s_preset, 0);
        break;

    case 'Z':   // 设定当前位置，默认 0（最下端）
        v = 0;
        if(has_arg(p) && !parse_cm100(&p, &v)) { printf("ERR FORMAT\r\n"); break; }
        if(v < 0 || v > MAX_POS_CM100) { err_range(); break; }
        if(Stepper_IsBusy()) { printf("ERR BUSY\r\n"); break; }
        Stepper_SetPosition(cm100_to_steps(v));
        s_homed = 1;
        printf("OK Z ");
        print_cm(v);
        printf("\r\n");
        break;

    case 'V':   // 设定/查询速度（cm/s），运动中也立即平滑生效
        if(has_arg(p))
        {
            if(!parse_cm100(&p, &v)) { printf("ERR FORMAT\r\n"); break; }
            if(v < MIN_SPEED_CM100 || v > MAX_SPEED_CM100)
            {
                printf("ERR RANGE %d.%02d-%d.%02d\r\n",
                       MIN_SPEED_CM100 / 100, MIN_SPEED_CM100 % 100,
                       MAX_SPEED_CM100 / 100, MAX_SPEED_CM100 % 100);
                break;
            }
            Stepper_SetSpeedHz((u32)v * STEPS_PER_CM_X1000 / 100000UL);
        }
        printf("OK V ");
        print_cm(steps_to_cm100((s32)Stepper_GetSpeedHz()));
        printf("\r\n");
        break;

    case '?':
    case 'H':
        print_help();
        break;

    default:
        printf("ERR UNKNOWN\r\n");
        break;
    }
}

/* ================= 串口拆行 ================= */
static void poll_uart(void)
{
    static char buf[LINE_MAX + 1];
    static u8   len = 0;
    static u8   overflow = 0;
    int ch;

    while((ch = USART1_ReadByte()) >= 0)
    {
        if(ch == '\r' || ch == '\n')
        {
            if(overflow)     printf("ERR TOOLONG\r\n");
            else if(len > 0) { buf[len] = '\0'; handle_line(buf); }
            len = 0;
            overflow = 0;
        }
        else if(len < LINE_MAX) buf[len++] = (char)ch;
        else                    overflow = 1;
    }
}

int main(void)
{
    const char *rst;

    SysTick_Init(72);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    USART1_Init(115200);
    Stepper_Init();

    // 判断复位原因：只有上电复位时位置才可信（用户保证上电时在最下端）
    if(RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)       rst = "POR";
    else if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) rst = "WDG";
    else if(RCC_GetFlagStatus(RCC_FLAG_SFTRST) != RESET)  rst = "SFT";
    else if(RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)  rst = "PIN";
    else                                                  rst = "OTHER";
    s_homed = (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET) || HOMED_ON_ANY_RESET;
    RCC_ClearFlag();
    Stepper_SetPosition(0);

    printf("\r\nREADY MAX %d.00 RST %s%s\r\n", MAX_POS_CM, rst, s_homed ? "" : " UNHOMED");

    IWDG_Init(IWDG_Prescaler_64, 1250);   // 约 2s

    while(1)
    {
        IWDG_ReloadCounter();
        poll_uart();
        if(s_moving && !Stepper_IsBusy()) report_done();
    }
}
