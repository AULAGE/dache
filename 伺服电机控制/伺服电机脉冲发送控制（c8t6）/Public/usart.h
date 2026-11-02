#ifndef __usart_H
#define __usart_H

#include "system.h"

void USART1_Init(u32 bound);
int  USART1_ReadByte(void);   // 无数据返回 -1

#endif
