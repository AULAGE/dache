#ifndef __BEEP_H
#define __BEEP_H

#include "system.h"

#define BEEP_PIN      GPIO_Pin_8
#define BEEP_PORT     GPIOB
#define BEEP_CLK      RCC_APB2Periph_GPIOB

void BEEP_Init(void);
void BEEP_ON(void);
void BEEP_OFF(void);

#endif
