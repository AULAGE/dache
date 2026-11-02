#include "usart.h"
#include <stdio.h>

// ========== printf 重定向到 USART1（Keil 勾选 Use MicroLIB） ==========
#pragma import(__use_no_semihosting)
struct __FILE { int handle; };
FILE __stdout;
void _sys_exit(int x) { x = x; }

int fputc(int ch, FILE *f)
{
    while(!(USART1->SR & USART_FLAG_TXE));
    USART1->DR = (u8)ch;
    return ch;
}

// ========== 接收环形缓冲 ==========
// 中断只负责把字节放进缓冲区，拆行交给主循环，命令连发也不会丢
#define RX_RING_SIZE 256   // 必须是 2 的幂
static volatile u8  s_ring[RX_RING_SIZE];
static volatile u16 s_head = 0;   // 中断写
static volatile u16 s_tail = 0;   // 主循环读

void USART1_Init(u32 bound)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef us;
    NVIC_InitTypeDef  nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_9;            // TX
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin   = GPIO_Pin_10;           // RX
    gpio.GPIO_Mode  = GPIO_Mode_IPU;         // 上拉：线没接时不会收到乱码
    GPIO_Init(GPIOA, &gpio);

    us.USART_BaudRate            = bound;
    us.USART_WordLength          = USART_WordLength_8b;
    us.USART_StopBits            = USART_StopBits_1;
    us.USART_Parity              = USART_Parity_No;
    us.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    us.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &us);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    nvic.NVIC_IRQChannel                   = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority        = 3;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

void USART1_IRQHandler(void)
{
    u16 sr = USART1->SR;
    u8  ch;
    u16 next;

    // 读 SR 再读 DR：同时清 RXNE 和 ORE/FE/NE 错误标志
    if(sr & (USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_NE | USART_FLAG_PE))
    {
        ch = (u8)USART1->DR;
        if((sr & USART_FLAG_RXNE) && !(sr & (USART_FLAG_FE | USART_FLAG_NE | USART_FLAG_PE)))
        {
            next = (s_head + 1) & (RX_RING_SIZE - 1);
            if(next != s_tail)          // 满了就丢弃
            {
                s_ring[s_head] = ch;
                s_head = next;
            }
        }
    }
}

int USART1_ReadByte(void)
{
    u8 ch;
    if(s_tail == s_head) return -1;
    ch = s_ring[s_tail];
    s_tail = (s_tail + 1) & (RX_RING_SIZE - 1);
    return ch;
}
