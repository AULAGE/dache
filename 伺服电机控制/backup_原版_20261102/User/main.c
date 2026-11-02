#include "system.h"
#include "SysTick.h"
#include "usart.h"
#include "stepper.h"
#include "iwdg.h"
#include <stdio.h>

static u8 was_busy = 0;
static s32 saved_pos_steps = 0;

// ★ 步数转厘米（带1位小数），返回值单位 = 0.1cm
s32 steps_to_cm_x10(s32 steps)
{
    return steps * 10000L / (s32)STEPS_PER_CM_X1000;
}

// ★ 打印位置（统一格式）
void print_pos(const char *prefix)
{
    s32 pos  = Stepper_GetPosition();
    s32 cm10 = steps_to_cm_x10(pos);
    s32 abs10 = (cm10 < 0) ? -cm10 : cm10;
    printf("%s%s%d.%dcm\r\n", prefix,
           (cm10 < 0) ? "-" : "",
           abs10 / 10, abs10 % 10);
}

void parse_and_execute(u8 *buf, u16 len)
{
    u8  dir = 0;
    u32 distance_cm = 0;
    u32 total_steps;
    u16 i = 0;
		if(len == 0) return; 
    // 急停
    if(buf[0] == 'S' || buf[0] == 's')
    {
        Stepper_Stop();
				was_busy = 0;   
        print_pos("STOP! pos=");
        return;
    }
		 // ★ 设定目标位置：M 20
		 if(buf[0] == 'M' || buf[0] == 'm')
		{
        u32 target_cm = 0;
        u16 j = 1;
        s32 cm10;
				u8 has_digit = 0;
        while(j < len && buf[j] == ' ') j++;
        while(j < len && buf[j] >= '0' && buf[j] <= '9')
        {
						has_digit = 1;
            target_cm = target_cm * 10 + (buf[j] - '0');
            j++;
        }
				if(!has_digit)
				{
				printf("ERR: format -> M 20\r\n");
				return;
				}
        #if (MIN_POS_CM > 0)
				if(target_cm < MIN_POS_CM || target_cm > MAX_POS_CM)
				#else
				if(target_cm > MAX_POS_CM)
				#endif
				{
				printf("ERR: range %d~%dcm\r\n", MIN_POS_CM, MAX_POS_CM);
				return;
				}
        saved_pos_steps = (s32)(target_cm * STEPS_PER_CM_X1000 / 1000);
        cm10 = steps_to_cm_x10(saved_pos_steps);
        printf("SET: target=%d.%dcm\r\n", cm10/10, cm10%10);
        return;
    }
		if(buf[0] == 'G' || buf[0] == 'g')
    {
				s32 new_pos; 
        s32 cur, diff, cm10;
        u32 steps;
        u8  go_dir;
        if(Stepper_IsBusy())
        {
            printf("BUSY!\r\n");
            return;
        }
        cur  = Stepper_GetPosition();
        diff = saved_pos_steps - cur;
        if(diff == 0)
        {
            print_pos("Already at ");
            return;
        }
        if(diff > 0)
        {
            go_dir = 1;
            steps  = (u32)diff;
        }
        else
        {
            go_dir = 0;
            steps  = (u32)(-diff);
        }
        cm10 = steps_to_cm_x10(saved_pos_steps);
        printf("GOTO: %d.%dcm (%s %d steps)\r\n",
               cm10/10, cm10%10,
               go_dir ? "UP" : "DOWN", steps);
				new_pos = cur + (go_dir ? (s32)steps : -(s32)steps);
				if(new_pos > (s32)MAX_POS_STEPS)
			{
					steps = (u32)((s32)MAX_POS_STEPS - cur);
					printf("CLAMP: limited to upper bound\r\n");
			}
			if(new_pos < (s32)MIN_POS_STEPS)
			{
					steps = (u32)(cur - (s32)MIN_POS_STEPS);
					printf("CLAMP: limited to lower bound\r\n");
			}
			if(steps == 0)
			{
					printf("ERR: at limit\r\n");
					return;
			}
        Stepper_Move(go_dir, steps);
        was_busy = 1;
        return;
    }
    // 查询位置
    if(buf[0] == 'P' || buf[0] == 'p')
    {
        print_pos("POS: ");
        printf("  range: %d~%dcm  %s\r\n",
               MIN_POS_CM, MAX_POS_CM,
               Stepper_IsBusy() ? "(moving)" : "(idle)");
        return;
    }

    // 位置归零
    if(buf[0] == 'Z' || buf[0] == 'z')
    {
        if(Stepper_IsBusy())
        {
            printf("ERR: stop first\r\n");
            return;
        }
        Stepper_ResetPosition();
        printf("POS reset to 0cm\r\n");
        return;
    }

    // 忙检查
    if(Stepper_IsBusy())
    {
        printf("BUSY!\r\n");
        return;
    }

    // 解析方向
    while(i < len && buf[i] == ' ') i++;
    if(i < len && (buf[i] == '0' || buf[i] == '1'))
    {
        dir = buf[i] - '0';
        i++;
    }
    else
    {
        printf("ERR: format -> 1 10\r\n");
        return;
    }

    // 解析距离
    while(i < len && buf[i] == ' ') i++;
    while(i < len && buf[i] >= '0' && buf[i] <= '9')
    {
        distance_cm = distance_cm * 10 + (buf[i] - '0');
        i++;
    }
    if(distance_cm == 0)
    {
        printf("ERR: distance=0\r\n");
        return;
    }
    if(distance_cm > 1000)
    {
        printf("ERR: max 1000cm\r\n");
        return;
    }

    total_steps = (distance_cm * STEPS_PER_CM_X1000) / 1000;

    // ★★★ 限位检查 ★★★
     {
        s32 cur = Stepper_GetPosition();
        s32 room;
        s32 cm10;
        if(dir == 1)
        {
            room = (s32)MAX_POS_STEPS - cur;
            if(room <= 0)
            {
                printf("ERR: already at upper limit %dcm\r\n", MAX_POS_CM);
                return;
            }
            if((s32)total_steps > room)
            {
                total_steps = (u32)room;
                cm10 = steps_to_cm_x10(room);
                printf("CLAMP: only %d.%dcm to limit\r\n", cm10/10, cm10%10);
            }
        }
        else
        {
            room = cur - (s32)MIN_POS_STEPS;
            if(room <= 0)
            {
                printf("ERR: already at lower limit %dcm\r\n", MIN_POS_CM);
                return;
            }
            if((s32)total_steps > room)
            {
                total_steps = (u32)room;
                cm10 = steps_to_cm_x10(room);
                printf("CLAMP: only %d.%dcm to limit\r\n", cm10/10, cm10%10);
            }
        }
    }

    // 显示并执行
    {
        s32 cm10 = steps_to_cm_x10(total_steps);
        printf("GO: %s %d.%dcm (%d steps)\r\n",
               dir ? "UP" : "DOWN",
               cm10/10, cm10%10,
               total_steps);
    }

    Stepper_Move(dir, total_steps);
    was_busy = 1;
}

int main(void)
{
    u16 len;

    SysTick_Init(72);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    USART1_Init(115200);
    Stepper_Init();
		IWDG_Init(IWDG_Prescaler_64, 1250);

    printf("\r\n== Stepper Controller ==\r\n");
    printf("Range: %d~%dcm\r\n", MIN_POS_CM, MAX_POS_CM);
    printf("CMD:\r\n");
    printf("  1 10 - up 10cm\r\n");
    printf("  0 5  - down 5cm\r\n");
    printf("  S    - stop\r\n");
    printf("  P    - position\r\n");
    printf("  Z    - zero reset\r\n");
		printf("  M 20 - set target 20cm\r\n");
		printf("  G    - go to target\r\n");

    while(1)
    {
				IWDG_ReloadCounter();
        // 运动完成通知
        if(was_busy && !Stepper_IsBusy())
        {
            was_busy = 0;
						saved_pos_steps = Stepper_GetPosition();
            print_pos("DONE! pos=");
        }

        if(USART1_RX_STA & 0x8000)
        {
            len = USART1_RX_STA & 0x3FFF;
            parse_and_execute((u8 *)USART1_RX_BUF, len);
            USART1_RX_STA = 0;
        }
    }
}
