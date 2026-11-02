#ifndef __Key_H
#define __Key_H

#include "system.h"

#define BTN_PIN     GPIO_Pin_14
#define BTN_PORT    GPIOB
#define BTN_CLK     RCC_APB2Periph_GPIOB

void Key_Init(void);
u8   Key_Read(void);
u8   Key_GetPressed(void);

#endif
