#include "usart2.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"

// 初始化USART2
void UART2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // 开启时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  // 配置USART2引脚 (PA2-TX, PA3-RX)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // TX引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; // RX引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // 配置USART2参数
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  
  USART_Init(USART2, &USART_InitStructure);
  
  // 使能USART2接收中断
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART2, ENABLE);

  // 配置USART2中断
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

// USART2中断服务函数 - 接收数据并转发到USART1
void USART2_IRQHandler(void)
{
  // 检查是否是接收中断
  if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
    // 读取接收到的字节
    uint8_t receivedData = USART_ReceiveData(USART2);
    
    // 将数据通过USART1发送出去
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // 等待发送寄存器空
    USART_SendData(USART1, receivedData);
  }
  
  // 清除中断标志
  USART_ClearITPendingBit(USART2, USART_IT_RXNE);
}
